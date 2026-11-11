"""Opt-in Fast DDS camera payload -> project CycloneDDS loopback regression.

Run with B2_CAMERA_RUN_DDS_INTEROP=1 on Linux with both RMW packages installed.
Only synthetic Image/CameraInfo publishers run; the B2 SDK is never imported.
"""

import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1920, 1080
PAYLOAD = bytes([17, 83, 149]) * (WIDTH * HEIGHT)
FRAME = "f_oc_link"


def _endpoint(role, topic):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("camera_dds_" + role + "_" + topic.rsplit("/", 1)[-1])
    images, infos, paired = {}, {}, set()

    def key(message):
        return message.header.stamp.sec, message.header.stamp.nanosec

    def pair(stamp):
        if stamp not in images or stamp not in infos:
            return
        image, info = images.pop(stamp), infos.pop(stamp)
        assert image.header.frame_id == info.header.frame_id == FRAME
        assert image.width == info.width == WIDTH and image.height == info.height == HEIGHT
        assert image.encoding == "bgr8" and image.step == WIDTH * 3
        assert bytes(image.data) == PAYLOAD
        assert info.k[0] == info.k[4] == 500.0
        paired.add(stamp)

    def image_received(message):
        images[key(message)] = message
        pair(key(message))
        while len(images) > 10:
            images.pop(next(iter(images)))

    def info_received(message):
        infos[key(message)] = message
        pair(key(message))
        while len(infos) > 10:
            infos.pop(next(iter(infos)))

    if role == "pub":
        image_pub = node.create_publisher(Image, topic + "/image_raw", qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, topic + "/camera_info", qos_profile_sensor_data)

        def publish():
            image = Image()
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = FRAME
            image.width, image.height = WIDTH, HEIGHT
            image.encoding, image.step, image.data = "bgr8", WIDTH * 3, PAYLOAD
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = WIDTH, HEIGHT
            # Synthetic test calibration, never used by a physical camera.
            info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
            image_pub.publish(image)
            info_pub.publish(info)

        node.create_timer(0.2, publish)
    else:
        node.create_subscription(Image, topic + "/image_raw", image_received, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, topic + "/camera_info", info_received, qos_profile_sensor_data)
    deadline = time.monotonic() + (15 if role == "pub" else 10)
    try:
        while rclpy.ok() and time.monotonic() < deadline and len(paired) < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if role == "sub":
        print(f"PAIRED_CAMERA_FRAMES={len(paired)} PAYLOAD_BYTES={len(PAYLOAD)}", flush=True)
        return 0 if len(paired) >= 3 else 2
    return 0


def test_camera_payload_crosses_fastdds_to_project_cyclonedds(tmp_path):
    if os.environ.get("B2_CAMERA_RUN_DDS_INTEROP") != "1" or not sys.platform.startswith("linux"):
        pytest.skip("opt-in Linux DDS interop test")
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    except ImportError:
        pytest.skip("ROS package discovery is required")
    try:
        for package in ("rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"):
            get_package_prefix(package)
    except PackageNotFoundError:
        pytest.skip("both ROS RMW implementations are required")
    profile = Path(os.environ.get("B2_CAMERA_FASTDDS_PROFILE", PACKAGE / "config/fastdds.xml"))
    cyclone = Path(os.environ.get("B2_CAMERA_CYCLONEDDS_XML", (
        PACKAGE.parent.parent / "bringup/robot_bringup/config/cyclonedds.xml")))
    assert profile.is_file() and cyclone.is_file(), "camera/project DDS profiles are required"
    environment = dict(os.environ)
    for name in ("ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER",
                 "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        environment.pop(name, None)
    environment.update(ROS_DOMAIN_ID="219", ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET",
                       ROS_HOME=str(tmp_path / "ros"), ROS_LOG_DIR=str(tmp_path / "log"))
    topic = "/b2_camera_dds_test/sample_" + uuid.uuid4().hex
    publisher = subprocess.Popen(
        [sys.executable, __file__, "pub", topic], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True,
        env=dict(environment, RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                 RMW_FASTRTPS_USE_QOS_FROM_XML="1",
                 FASTRTPS_DEFAULT_PROFILES_FILE=str(profile)),
    )
    try:
        subscriber = subprocess.run(
            [sys.executable, __file__, "sub", topic], capture_output=True, text=True, timeout=14,
            env=dict(environment, RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
                     CYCLONEDDS_URI=cyclone.resolve().as_uri()),
        )
    finally:
        publisher.terminate()
        try:
            publisher_output = publisher.communicate(timeout=3)[0]
        except subprocess.TimeoutExpired:
            publisher.kill()
            publisher_output = publisher.communicate()[0]
    assert subscriber.returncode == 0, subscriber.stdout + subscriber.stderr + publisher_output
    assert f"PAIRED_CAMERA_FRAMES=3 PAYLOAD_BYTES={WIDTH * HEIGHT * 3}" in subscriber.stdout
    print(subscriber.stdout.strip())


if __name__ == "__main__":
    raise SystemExit(_endpoint(sys.argv[1], sys.argv[2]))
