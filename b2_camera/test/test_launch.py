"""Exercise argument boundaries without loading the SDK or starting ROS nodes."""

import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            ResetLaunchConfigurations)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("b2_camera_launch", PACKAGE / "launch/camera.launch.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def context(**overrides):
    result = LaunchContext()
    result.launch_configurations.update({
        "network_interface": "b2nic", "camera_info_url": "file:///measured/front.yaml", **overrides,
    })
    for action in MODULE.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(result)
    return result


def settings(node, ctx):
    return evaluate_parameters(ctx, node._Node__parameters)[0]


def test_front_and_back_have_separate_default_frames_and_topics(monkeypatch):
    monkeypatch.setattr(MODULE, "FindPackageShare", lambda name: TextSubstitution(text=str(PACKAGE)))
    front_context = context()
    back_context = context(camera_side="back")
    front, = MODULE._setup(front_context)
    back, = MODULE._setup(back_context)
    assert front.node_package == back.node_package == "b2_camera"
    assert settings(front, front_context)["frame_id"] == "f_oc_link"
    assert settings(back, back_context)["frame_id"] == "r_oc_link"
    assert front._Node__node_namespace == "b2/front_camera"
    assert back._Node__node_namespace == "b2/back_camera"
    executable = front._ExecuteLocal__process_description
    environment = {perform_substitutions(front_context, key): perform_substitutions(front_context, value)
                   for key, value in executable._Executable__additional_env}
    assert environment == {
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "RMW_FASTRTPS_USE_QOS_FROM_XML": "1",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(PACKAGE / "config/fastdds.xml"),
    }


def test_front_and_back_can_share_one_launch_without_argument_leaks(monkeypatch):
    records = []
    monkeypatch.setattr(MODULE, "FindPackageShare", lambda name: TextSubstitution(text=str(PACKAGE)))
    monkeypatch.setattr(PythonLaunchDescriptionSource, "get_launch_description",
                        lambda self, ctx: MODULE.generate_launch_description())

    def capture(node, ctx):
        records.append((node._Node__node_namespace, settings(node, ctx)))
        return []

    monkeypatch.setattr(MODULE.Node, "execute", capture)
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        GroupAction(actions=[ResetLaunchConfigurations(), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(PACKAGE / "launch/camera.launch.py")),
            launch_arguments={
                "network_interface": "b2nic", "camera_side": side,
                "camera_info_url": f"file:///measured/{side}.yaml",
            }.items(),
        )]) for side in ("front", "back")
    ]))
    assert service.run() == 0
    assert [(namespace, params["camera_side"], params["frame_id"], params["camera_info_url"])
            for namespace, params in records] == [
        ("b2/front_camera", "front", "f_oc_link", "file:///measured/front.yaml"),
        ("b2/back_camera", "back", "r_oc_link", "file:///measured/back.yaml"),
    ]


def test_ros_loopback_profile_is_node_local_and_disables_other_transports(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/parent/unchanged.xml")
    MODULE._setup(context())
    assert os.environ["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] == "/parent/unchanged.xml"
    root = ET.parse(PACKAGE / "config/fastdds.xml").getroot()
    namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    transport, = root.findall("f:transport_descriptors/f:transport_descriptor", namespace)
    assert transport.findtext("f:type", namespaces=namespace) == "UDPv4"
    addresses = transport.findall("f:interfaceWhiteList/f:address", namespace)
    assert [address.text for address in addresses] == ["127.0.0.1"]
    participant, = root.findall("f:participant", namespace)
    assert participant.attrib["is_default_profile"] == "true"
    rtps = participant.find("f:rtps", namespace)
    assert rtps.findtext("f:useBuiltinTransports", namespaces=namespace) == "false"
    assert rtps.findtext("f:userTransports/f:transport_id", namespaces=namespace) == transport.findtext(
        "f:transport_id", namespaces=namespace)


def test_missing_calibration_is_rejected_before_node_creation(monkeypatch):
    monkeypatch.setattr(MODULE, "Node", lambda **kwargs: pytest.fail("node constructed before calibration validation"))
    with pytest.raises(RuntimeError, match="camera_info_url"):
        MODULE._setup(context(camera_info_url=""))


def test_only_explicit_collection_mode_allows_missing_calibration():
    ctx = context(camera_info_url="", require_calibration="false")
    node, = MODULE._setup(ctx)
    assert settings(node, ctx)["require_calibration"] is False
    assert settings(node, ctx)["camera_info_url"] == ""


@pytest.mark.parametrize("override", [
    {"network_interface": ""}, {"publish_rate_hz": "0"}, {"publish_rate_hz": "nan"},
    {"publish_rate_hz": "1e-300"},
    {"publish_rate_hz": "inf"}, {"rpc_timeout_s": "0"}, {"rpc_timeout_s": "-1"},
    {"rpc_timeout_s": "inf"}, {"rpc_timeout_s": "11"}, {"dds_domain": "-1"},
    {"dds_domain": "233"}, {"dds_domain": "0.5"}, {"frame_id": "/bad_frame"},
])
def test_invalid_connection_and_unbounded_poll_settings_are_rejected(override):
    with pytest.raises(RuntimeError):
        MODULE._setup(context(**override))
