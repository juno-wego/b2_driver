# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

"""Actual front/back SDK camera processes against video-only loopback peers."""

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
WIDTH, HEIGHT = 1920, 1080

# Calibration tools use the same RMW as the camera's service server. Keep the
# image subscriber in the caller's RMW so CycloneDDS image interop stays tested.
SAVE_CALIBRATION = """
import sys
import rclpy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import yaml

document = yaml.safe_load(open(sys.argv[2]))
rclpy.init()
node = rclpy.create_node('sdk_calibration_writer')
try:
    client = node.create_client(SetCameraInfo, sys.argv[1])
    assert client.wait_for_service(timeout_sec=3.0), 'Calibration service unavailable'
    info = CameraInfo(
        width=document['image_width'], height=document['image_height'],
        distortion_model=document['distortion_model'],
        k=document['camera_matrix']['data'], d=document['distortion_coefficients']['data'],
        r=document['rectification_matrix']['data'], p=document['projection_matrix']['data'])
    future = client.call_async(SetCameraInfo.Request(camera_info=info))
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    assert future.done(), 'Calibration service timed out'
    assert future.result().success, future.result().status_message
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


@pytest.mark.parametrize('rate', [5.0, 30.0])
def test_actual_sdk_front_and_back_are_independent_and_reject_bad_samples(
    tmp_path, monkeypatch, rate
):
    peer = os.environ.get('B2_CAMERA_FAKE_VIDEO_PEER', '')
    camera = os.environ.get('B2_CAMERA_DRIVER_EXECUTABLE', '')
    if not Path(peer).is_file() or not Path(camera).is_file():
        pytest.skip('Built SDK camera and loopback video peer are required')
    monkeypatch.setenv(
        'CYCLONEDDS_URI', (ROOT / 'robot/bringup/robot_bringup/config/cyclonedds.xml').as_uri()
    )
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    expected = {}
    for side, color in [('front', (17, 83, 149)), ('back', (137, 41, 13))]:
        image = np.full((HEIGHT, WIDTH, 3), color, dtype=np.uint8)
        assert cv2.imwrite(str(tmp_path / f'{side}.jpg'), image)
        expected[side] = cv2.imread(str(tmp_path / f'{side}.jpg'))[HEIGHT // 2, WIDTH // 2]
        (tmp_path / f'{side}.control').write_text('good')
    assert cv2.imwrite(str(tmp_path / 'wrong.jpg'), np.zeros((540, 960, 3), dtype=np.uint8))
    imported_front = PACKAGE / 'config/calibration/b2_front_1920x1080.yaml'
    front_calibration = tmp_path / 'b2_front_camera.yaml'
    front_calibration.write_text('# SDK TEST COPY ONLY\n' + imported_front.read_text())
    front = yaml.safe_load(front_calibration.read_text())
    # Independent synthetic rear intrinsics only for this fixture. This is not
    # a calibration profile to install on any physical B2 camera.
    back = yaml.safe_load(front_calibration.read_text())
    back['camera_name'] = 'b2_back_camera'
    back['camera_matrix']['data'][0] = back['projection_matrix']['data'][0] = 601.0
    back_calibration = tmp_path / 'b2_back_camera.yaml'
    back_calibration.write_text('# SYNTHETIC BACK LENS ONLY\n' + yaml.safe_dump(back))
    env = dict(
        os.environ,
        ROS_DOMAIN_ID='220',
        ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET',
        ROS_LOG_DIR=str(tmp_path / 'ros_logs'),
        ROS_HOME=str(tmp_path / 'ros_home'),
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=os.environ.get(
            'B2_CAMERA_FASTDDS_PROFILE', str(PACKAGE / 'config/fastdds.xml')
        ),
        RMW_FASTRTPS_USE_QOS_FROM_XML=os.environ.get('B2_CAMERA_TEST_XML_QOS', '1'),
    )
    processes, logs = [], []
    context = Context()
    rclpy.init(context=context, domain_id=220)
    node = rclpy.create_node('sdk_video_runtime_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = {
        side: {'images': {}, 'infos': {}, 'pairs': [], 'ages': []} for side in ('front', 'back')
    }

    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def receive(side, kind, message):
        data = received[side]
        key = stamp(message)
        assert message.header.frame_id == ('f_oc_link' if side == 'front' else 'r_oc_link')
        assert (message.width, message.height) == (WIDTH, HEIGHT)
        if kind == 'images':
            assert message.encoding == 'bgr8' and message.step == WIDTH * 3
            offset = (HEIGHT // 2) * message.step + (WIDTH // 2) * 3
            assert np.array_equal(np.array(message.data[offset:offset + 3]), expected[side])
            value = message.header
        else:
            calibration = front if side == 'front' else back
            assert list(message.k) == calibration['camera_matrix']['data']
            assert list(message.d) == calibration['distortion_coefficients']['data']
            assert list(message.p) == calibration['projection_matrix']['data']
            assert message.distortion_model == 'equidistant'
            value = message.header
        data[kind][key] = value
        if key in data['images'] and key in data['infos']:
            assert data['images'].pop(key) == data['infos'].pop(key)
            assert not data['pairs'] or key > data['pairs'][-1]
            data['pairs'].append(key)
            data['ages'].append((node.get_clock().now().nanoseconds - key) * 1e-9)
        while len(data[kind]) > 10:
            data[kind].pop(next(iter(data[kind])))

    for side in ('front', 'back'):
        for kind, msg_type, topic in [
            ('images', Image, 'image_raw'),
            ('infos', CameraInfo, 'camera_info'),
        ]:
            node.create_subscription(
                msg_type,
                f'/synthetic_sdk/{side}/{topic}',
                lambda m, s=side, k=kind: receive(s, k, m),
                qos_profile_sensor_data,
            )

    def diagnostics():
        return '\n'.join(path.read_text() for path, _ in logs)

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), diagnostics()
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), diagnostics()

    def advance(seconds):
        deadline = time.monotonic() + seconds
        spin_until(lambda: time.monotonic() >= deadline, seconds + 0.5)

    def start(command, name, environment):
        path = tmp_path / f'{name}.log'
        log = path.open('w+')
        logs.append((path, log))
        processes.append(
            subprocess.Popen(
                command,
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        )

    def mode(value):
        pending = tmp_path / 'front.pending'
        pending.write_text(value)
        pending.replace(tmp_path / 'front.control')

    completed = False
    try:
        start([peer, str(tmp_path)], 'peer', env)
        spin_until(lambda: 'LOOPBACK_VIDEO_READY' in (tmp_path / 'peer.log').read_text(), 5.0)
        for side, calibration in [('front', front_calibration), ('back', back_calibration)]:
            start(
                [
                    camera,
                    '--ros-args',
                    '-r',
                    f'__ns:=/synthetic_sdk/{side}',
                    '-r',
                    '~/set_camera_info:=set_camera_info',
                    '-p',
                    'network_interface:=lo',
                    '-p',
                    f'camera_side:={side}',
                    '-p',
                    'dds_domain:=112',
                    '-p',
                    f'publish_rate_hz:={rate}',
                    '-p',
                    'rpc_timeout_s:=0.2',
                    '-p',
                    f'camera_info_url:={calibration.as_uri()}',
                    '-p',
                    f'frame_id:={"f_oc_link" if side == "front" else "r_oc_link"}',
                ],
                side,
                env,
            )
        spin_until(lambda: all(len(data['pairs']) >= 3 for data in received.values()), 15.0)
        for side, calibration, document, other in [
            ('front', front_calibration, front, back_calibration),
            ('back', back_calibration, back, front_calibration),
        ]:
            other_bytes = other.read_bytes()
            before = calibration.read_bytes()
            saved_log = tmp_path / f'{side}_save.log'
            with saved_log.open('w') as output:
                writer = subprocess.Popen(
                    [
                        '/usr/bin/python3',
                        '-c',
                        SAVE_CALIBRATION,
                        f'/synthetic_sdk/{side}/set_camera_info',
                        str(calibration),
                    ],
                    env=dict(
                        env,
                        RMW_FASTRTPS_USE_QOS_FROM_XML='1',
                        RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',
                    ),
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    spin_until(lambda: writer.poll() is not None, 8.0)
                    assert writer.returncode == 0, saved_log.read_text()
                finally:
                    if writer.poll() is None:
                        os.killpg(writer.pid, signal.SIGTERM)
                        writer.wait(timeout=3.0)
            after = yaml.safe_load(calibration.read_text())
            assert calibration.read_bytes() != before  # The service actually saved this URL.
            assert after['camera_name'] == f'b2_{side}_camera'
            assert after['camera_matrix']['data'] == document['camera_matrix']['data']
            assert other.read_bytes() == other_bytes  # No peer-file overwrite.
        assert not (tmp_path / 'ros_home/camera_info').exists()
        initial = {side: len(data['pairs']) for side, data in received.items()}
        advance(3.0)
        for side, data in received.items():
            assert len(data['pairs']) - initial[side] >= 6, (side, diagnostics())
        for failure in ('corrupt', 'empty', 'error', 'mismatch', 'silence'):
            mode(failure)
            advance(0.7)  # Discard an in-flight good RPC/queued DDS pair.
            before_front, before_back = [len(received[s]['pairs']) for s in ('front', 'back')]
            advance(1.2)
            assert len(received['front']['pairs']) == before_front, (failure, diagnostics())
            assert len(received['back']['pairs']) > before_back, (failure, diagnostics())
            mode('good')
            spin_until(lambda: len(received['front']['pairs']) >= before_front + 2, 5.0)
        completed = True
    finally:
        # Clients stop before their SDK peer; no pending camera RPC is abandoned
        # by deliberately removing the server first.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        context.try_shutdown()
        for _, log in logs:
            log.close()
    if completed:
        assert all(p.returncode == 0 for p in processes), diagnostics()
        assert 'unexpected=0' in (tmp_path / 'peer.log').read_text(), diagnostics()
        for side, data in received.items():
            assert max(data['ages']) < 0.5, (side, rate, data['ages'])
            assert np.quantile(data['ages'], 0.95) < 0.3, (side, rate, data['ages'])
        metrics = {
            side: {
                'pairs': len(data['pairs']),
                'max_receipt_to_ros_age_s': max(data['ages']),
                'p95_receipt_to_ros_age_s': float(np.quantile(data['ages'], 0.95)),
            }
            for side, data in received.items()
        }
        (tmp_path / 'metrics.json').write_text(
            json.dumps(
                {
                    'synthetic_only': True,
                    'requested_rate_hz': rate,
                    'payload_bytes': WIDTH * HEIGHT * 3,
                    'streams': metrics,
                    'driver_sha256': hashlib.sha256(Path(camera).read_bytes()).hexdigest(),
                    'fastdds_profile_sha256': hashlib.sha256(
                        Path(env['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes()
                    ).hexdigest(),
                },
                indent=2,
            )
            + '\n'
        )
        print('SYNTHETIC_SDK_VIDEO', rate, metrics)


@pytest.mark.parametrize(
    'side,supplied',
    [
        ('front', 'back'),
        ('back', 'front'),
        ('back', 'package_front'),
        ('back', 'template_back'),
    ],
)
def test_wrong_side_or_unmeasured_calibration_is_rejected_before_sdk(tmp_path, side, supplied):
    camera = os.environ.get('B2_CAMERA_DRIVER_EXECUTABLE', '')
    if not Path(camera).is_file():
        pytest.skip('Built SDK camera is required')
    if supplied == 'package_front':
        url = 'package://b2_camera/config/calibration/b2_front_1920x1080.yaml'
    elif supplied == 'template_back':
        url = (PACKAGE / 'config/calibration/b2_back_camera.example.yaml').as_uri()
    else:
        document = yaml.safe_load(
            (PACKAGE / 'config/calibration/b2_front_1920x1080.yaml').read_text()
        )
        document['camera_name'] = f'b2_{supplied}_camera'
        calibration = tmp_path / 'wrong_side.yaml'
        calibration.write_text(yaml.safe_dump(document))
        url = calibration.as_uri()
    result = subprocess.run(
        [
            camera,
            '--ros-args',
            '-p',
            'network_interface:=lo',
            '-p',
            f'camera_side:={side}',
            '-p',
            f'frame_id:={"f_oc_link" if side == "front" else "r_oc_link"}',
            '-p',
            'dds_domain:=112',
            '-p',
            f'camera_info_url:={url}',
        ],
        env=dict(
            os.environ,
            ROS_DOMAIN_ID='220',
            RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            ROS_LOG_DIR=str(tmp_path / 'ros_logs'),
        ),
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    assert result.returncode != 0, result.stdout + result.stderr
    if supplied != 'template_back':
        assert f"calibration camera_name must be 'b2_{side}_camera'" in result.stderr
    else:
        assert 'failed to read measured camera calibration' in result.stderr
    assert 'GetImageSample' not in result.stdout + result.stderr
