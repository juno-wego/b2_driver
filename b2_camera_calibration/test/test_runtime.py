"""Actual upstream fisheye fitting, GUI model selection and isolated saved data."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import threading
import time
from unittest.mock import patch

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from b2_camera_calibration.calibrator import B2CalibrationNode, NamedMonoCalibrator
from camera_calibration.calibrator import CAMERA_MODEL, ChessboardInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.srv import SetCameraInfo
import yaml


@pytest.fixture
def ros_context():
    if not os.environ.get('DISPLAY'):
        pytest.skip('Run GUI checks under xvfb-run or an available display')
    rclpy.init()
    yield
    rclpy.try_shutdown()
    cv2.destroyAllWindows()


@pytest.mark.parametrize('side', ['front', 'back'])
def test_gui_receives_best_effort_raw_frames_and_selects_fisheye_immediately(
    ros_context, tmp_path, side
):
    node = B2CalibrationNode(side, (8, 6), 0.025, 'fisheye', tmp_path)
    source = rclpy.create_node('calibration_frame_source')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(source)
    # Remap the input exactly as the launch does using the node's resolved name.
    topic = node.resolve_topic_name('image')
    publisher = source.create_publisher(Image, topic, qos_profile_sensor_data)
    frame = CvBridge().cv2_to_imgmsg(np.full((480, 640), 255, np.uint8), encoding='mono8')
    deadline = time.monotonic() + 5.0
    try:
        while node.c is None and time.monotonic() < deadline:
            publisher.publish(frame)
            executor.spin_once(timeout_sec=0.02)
        assert node.c is not None
        assert node.c.camera_model == CAMERA_MODEL.FISHEYE
        assert node.c.name == f'b2_{side}_camera'
        # Exit the real GUI loop through its normal q handler and join its ROS worker.
        original = cv2.waitKey
        with patch.object(cv2, 'waitKey', return_value=ord('q')):
            node.spin()
        assert cv2.waitKey is original
    finally:
        executor.remove_node(node)
        executor.remove_node(source)
        executor.shutdown(timeout_sec=2.0)
        source.destroy_node()
        node.destroy_node()


def fit_synthetic_fisheye(name, root, focal):
    board = ChessboardInfo('chessboard', 8, 6, 0.025)
    calibrator = NamedMonoCalibrator([board], name, CAMERA_MODEL.FISHEYE, root)
    calibrator.size = (640, 480)
    k = np.array([[focal, 0, 320.0], [0, focal + 4.0, 240.0], [0, 0, 1.0]])
    d = np.array([-0.03, 0.004, -0.001, 0.0001])
    points = np.zeros((1, board.n_cols * board.n_rows, 3), np.float64)
    points[0, :, :2] = np.mgrid[: board.n_cols, : board.n_rows].T.reshape(-1, 2) * board.dim
    observations = []
    for index in range(18):
        rotation = np.array([0.12 * (index % 3 - 1), 0.14 * (index % 5 - 2), 0.03 * (index % 4)])
        translation = np.array(
            [-0.14 + 0.03 * (index % 6), -0.1 + 0.025 * (index % 5), 0.4 + 0.025 * (index % 4)]
        )
        pixels, _ = cv2.fisheye.projectPoints(points, rotation, translation, k, d)
        observations.append((pixels.reshape(-1, 1, 2), None, board))
    calibrator.cal_fromcorners(observations)
    np.testing.assert_allclose(calibrator.intrinsics, k, atol=0.1)
    np.testing.assert_allclose(calibrator.distortion.ravel(), d, atol=0.01)
    calibrator.calibrated = True  # The upstream GUI sets this after cal_fromcorners returns.
    return calibrator


def test_two_fits_save_separate_names_models_and_session_archives(tmp_path):
    front = fit_synthetic_fisheye('b2_front_camera', tmp_path, 320.0)
    back = fit_synthetic_fisheye('b2_back_camera', tmp_path, 350.0)
    front.do_save()
    front_file = tmp_path / 'camera_info/b2_front_camera.yaml'
    front_bytes = front_file.read_bytes()
    back.do_save()
    assert front_file.read_bytes() == front_bytes
    for name, focal in [('b2_front_camera', 320.0), ('b2_back_camera', 350.0)]:
        saved = yaml.safe_load((tmp_path / 'camera_info' / (name + '.yaml')).read_text())
        assert saved['camera_name'] == name and saved['distortion_model'] == 'equidistant'
        assert saved['camera_matrix']['data'][0] == pytest.approx(focal, abs=0.1)
        archives = list((tmp_path / 'datasets' / name).glob('*/calibrationdata.tar.gz'))
        assert len(archives) == 1
        with tarfile.open(archives[0]) as archive:
            assert yaml.safe_load(archive.extractfile('ost.yaml').read())['camera_name'] == name
        metadata = json.loads(archives[0].with_name('session.json').read_text())
        assert metadata['board'][0] == {'inner_corners': [8, 6], 'square_size_m': 0.025}
    front.do_save()
    assert len(list((tmp_path / 'datasets/b2_front_camera').glob('*/calibrationdata.tar.gz'))) == 2
    assert len(list((tmp_path / 'datasets/b2_back_camera').glob('*/calibrationdata.tar.gz'))) == 1


def test_uncalibrated_frames_cannot_be_saved_as_a_result(tmp_path):
    calibrator = NamedMonoCalibrator(
        [ChessboardInfo('chessboard', 8, 6, 0.025)],
        'b2_front_camera',
        CAMERA_MODEL.FISHEYE,
        tmp_path,
    )
    with pytest.raises(ValueError, match='Calibrate'):
        calibrator.do_save()
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('side', ['front', 'back'])
def test_gui_commit_uses_real_sdk_drivers_per_camera_save_service(ros_context, tmp_path, side):
    prefix = Path(get_package_prefix('b2_camera'))
    driver = prefix / 'lib/b2_camera/b2_camera_node'
    if not driver.is_file():
        pytest.skip('Build the B2 camera driver for the commit integration check')
    env = dict(
        os.environ,
        ROS_HOME=str(tmp_path),
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET',
        RMW_FASTRTPS_USE_QOS_FROM_XML='1',
        RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(
            Path(get_package_share_directory('b2_camera')) / 'config/fastdds.xml'
        ),
    )
    log = (tmp_path / 'camera.log').open('w')
    process = subprocess.Popen(
        [
            str(driver),
            '--ros-args',
            '-r',
            f'__ns:=/test/calibration/{side}',
            '-r',
            '~/set_camera_info:=set_camera_info',
            '-p',
            'network_interface:=lo',
            '-p',
            f'camera_side:={side}',
            '-p',
            f'frame_id:={"f_oc_link" if side == "front" else "r_oc_link"}',
            '-p',
            'dds_domain:=115',
            '-p',
            'rpc_timeout_s:=0.05',
            '-p',
            'require_calibration:=false',
        ],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    node = B2CalibrationNode(side, (8, 6), 0.025, 'fisheye', tmp_path)
    node.c = fit_synthetic_fisheye(f'b2_{side}_camera', tmp_path, 320.0)
    node.destroy_client(node.set_camera_info_service)
    node.set_camera_info_service = node.create_client(
        SetCameraInfo, f'/test/calibration/{side}/set_camera_info'
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert node.set_camera_info_service.wait_for_service(timeout_sec=5.0)
        assert node.do_upload()
        files = list((tmp_path / 'camera_info').glob('*.yaml'))
        assert [p.name for p in files] == [f'b2_{side}_camera.yaml']
        result = yaml.safe_load(files[0].read_text())
        assert result['camera_name'] == f'b2_{side}_camera'
        assert result['distortion_model'] == 'equidistant'
        assert result['camera_matrix']['data'][0] == pytest.approx(320.0, abs=0.1)
    finally:
        executor.shutdown(timeout_sec=2.0)
        worker.join(timeout=2.0)
        node.destroy_node()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        process.wait(timeout=3.0)
        log.close()
    assert process.returncode == 0


def test_commit_timeout_or_rejection_keeps_the_calibration_available(ros_context, tmp_path):
    node = B2CalibrationNode('front', (8, 6), 0.025, 'fisheye', tmp_path)
    node.c = fit_synthetic_fisheye('b2_front_camera', tmp_path, 320.0)
    try:
        assert not node.check_set_camera_info(
            SetCameraInfo.Response(success=False, status_message='Read-only target')
        )
        started = time.monotonic()
        assert not node.do_upload()
        assert 4.5 < time.monotonic() - started < 7.0
        assert node.c.calibrated
        assert not (tmp_path / 'camera_info').exists()
    finally:
        node.destroy_node()
