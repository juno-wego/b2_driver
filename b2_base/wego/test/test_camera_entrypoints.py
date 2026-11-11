# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

"""Expand the public camera entry points without starting hardware or a GUI."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import evaluate_parameters
import pytest


DRIVER = Path(__file__).resolve().parents[3]
SHARES = {
    'b2_base': DRIVER / 'b2_base/wego',
    'b2_camera': DRIVER / 'b2_camera',
    'b2_camera_calibration': DRIVER / 'b2_camera_calibration',
}


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


class Source(PythonLaunchDescriptionSource):
    def __init__(self, location):
        super().__init__(location)
        self.declared_location = location


@pytest.mark.parametrize('side', ['front', 'back'])
@pytest.mark.parametrize('calibrating', [False, True])
def test_public_entry_forwards_required_inputs_without_body_control(
    monkeypatch, tmp_path, side, calibrating,
):
    monkeypatch.syspath_prepend(str(SHARES['b2_camera_calibration']))
    from b2_camera_calibration import configuration

    monkeypatch.setattr(configuration, 'get_package_share_directory',
                        lambda name: str(SHARES[name]))
    records = []

    def load(path):
        spec = importlib.util.spec_from_file_location('camera_entry_' + path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', Source, raising=False)
        return module.generate_launch_description()

    def describe(source, context):
        return load(Path(resolve(context, source.declared_location)))

    def record(node, context):
        parameters = {}
        for item in evaluate_parameters(context, node._Node__parameters):
            if isinstance(item, dict):
                parameters.update(item)
        records.append((resolve(context, node.node_executable), parameters))
        return []

    def forbid_process(_action, _context):
        raise AssertionError('entry point tests must never start a process')

    monkeypatch.setattr(FindPackageShare, 'perform', lambda self, context:
                        str(SHARES[resolve(context, self.package)]))
    monkeypatch.setattr(Source, 'get_launch_description', describe)
    monkeypatch.setattr(Source, 'try_get_launch_description_without_context', lambda self: None)
    monkeypatch.setattr(ExecuteProcess, 'execute', forbid_process)
    monkeypatch.setattr(Node, 'execute', record)
    values = {
        'network_interface': 'test_camera_nic', 'camera_side': side,
        'camera_info_url': 'file:///measured/camera.yaml',
        'board_size': '8x6', 'square_size_m': '0.025', 'output_root': str(tmp_path),
    }
    filename = 'calibrate_camera.launch.py' if calibrating else 'camera.launch.py'
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        *(SetLaunchConfiguration(name, value) for name, value in values.items()),
        load(SHARES['b2_base'] / 'launch' / filename),
    ]))
    assert service.run() == 0
    assert sorted(name for name, _ in records) == (
        ['b2_camera_node', 'cameracalibrator'] if calibrating else ['b2_camera_node'])
    camera = next(params for name, params in records if name == 'b2_camera_node')
    assert camera['network_interface'] == 'test_camera_nic'
    assert camera['camera_side'] == side
    assert camera['frame_id'] == ('f_oc_link' if side == 'front' else 'r_oc_link')
    assert camera['require_calibration'] is (not calibrating)
    assert camera['camera_info_url'] == ('' if calibrating else 'file:///measured/camera.yaml')
