"""Expand raw capture/calibration wiring, including parent-argument isolation."""

import importlib.util
from pathlib import Path

from b2_camera_calibration.configuration import board_settings, default_output_root
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import evaluate_parameters
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
LAUNCH = PACKAGE / 'launch/calibrate.launch.py'


def test_default_results_belong_to_the_camera_driver_source_with_symlink_install(
    monkeypatch, tmp_path,
):
    from b2_camera_calibration import configuration
    source = tmp_path / 'source/b2_camera/config/calibration'
    installed = tmp_path / 'install/b2_camera/config/calibration'
    source.mkdir(parents=True)
    installed.mkdir(parents=True)
    (source / 'README.md').write_text('driver-owned measured calibration')
    (installed / 'README.md').symlink_to(source / 'README.md')
    monkeypatch.setattr(
        configuration, 'get_package_share_directory', lambda _: str(tmp_path / 'install/b2_camera'),
    )
    assert default_output_root() == str(source)


def load(path):
    spec = importlib.util.spec_from_file_location('calibration_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


@pytest.mark.parametrize('side', ['front', 'back'])
def test_capture_and_gui_share_side_output_and_fast_dds_without_inheriting_calibration(
    monkeypatch, tmp_path, side
):
    module = load(LAUNCH)
    real_node = module.Node
    gui_options, records = [], []

    def gui(**options):
        gui_options.append(options)
        return real_node(**options)

    def describe(source, context):
        path = Path(resolve(context, source._LaunchDescriptionSource__location))
        return load(path).generate_launch_description()

    def capture(node, context):
        parameters = {}
        for item in evaluate_parameters(context, node._Node__parameters):
            if isinstance(item, dict):
                parameters.update(item)
        executable = node._ExecuteLocal__process_description
        records.append(
            {
                'package': resolve(context, node.node_package),
                'namespace': resolve(context, node._Node__node_namespace),
                'parameters': parameters,
                'environment': {
                    resolve(context, k): resolve(context, v)
                    for k, v in executable._Executable__additional_env
                },
                'ros_home': context.environment.get('ROS_HOME'),
            }
        )
        return []

    monkeypatch.setattr(module, 'Node', gui)
    monkeypatch.setattr(Node, 'execute', capture)
    monkeypatch.setattr(PythonLaunchDescriptionSource, 'get_launch_description', describe)
    monkeypatch.setattr(
        FindPackageShare,
        'perform',
        lambda self, context: str(
            {
                'b2_camera': ROOT / 'robot/third_party/b2_driver/b2_camera',
            }[perform_substitutions(context, self.package)]
        ),
    )
    overrides = {
        'camera_side': side,
        'network_interface': 'b2nic',
        'board_size': '8x6',
        'square_size_m': '0.025',
        'output_root': str(tmp_path),
        'camera_info_url': 'file:///borrowed/reference.yaml',
        'require_calibration': 'true',
        'namespace': 'parent',
    }
    service = LaunchService(noninteractive=True)
    service.include_launch_description(
        LaunchDescription(
            [
                *(SetLaunchConfiguration(k, v) for k, v in overrides.items()),
                *module.generate_launch_description().entities,
            ]
        )
    )
    assert service.run() == 0
    assert len(records) == 2
    (camera,) = [r for r in records if r['package'] == 'b2_camera']
    (tool,) = [r for r in records if r['package'] == 'b2_camera_calibration']
    assert camera['namespace'] == f'b2/{side}_camera'
    assert camera['parameters']['camera_side'] == side
    assert camera['parameters']['camera_info_url'] == ''
    assert camera['parameters']['require_calibration'] is False
    assert camera['ros_home'] == str(tmp_path)
    for record in records:
        assert record['environment']['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        assert record['environment']['RMW_FASTRTPS_USE_QOS_FROM_XML'] == '1'
    (options,) = gui_options
    assert options['remappings'] == [
        ('image', f'/b2/{side}_camera/image_raw'),
        ('camera/set_camera_info', f'/b2/{side}_camera/set_camera_info'),
    ]
    assert options['additional_env']['ROS_HOME'] == str(tmp_path)
    assert options['arguments'] == [
        '--camera-side',
        side,
        '--board-size',
        '8x6',
        '--square-size-m',
        '0.025',
        '--camera-model',
        'fisheye',
        '--output-root',
        str(tmp_path),
    ]


@pytest.mark.parametrize(
    'size,square',
    [
        ('', '0.025'),
        ('8', '0.025'),
        ('8x2', '0.025'),
        ('0x6', '0.025'),
        ('8.0x6', '0.025'),
        ('8x6', ''),
        ('8x6', '0'),
        ('8x6', '-1'),
        ('8x6', 'nan'),
        ('8x6', 'inf'),
    ],
)
def test_bad_board_measurements_are_rejected_before_creating_any_node(monkeypatch, size, square):
    module = load(LAUNCH)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'network_interface': 'b2nic',
            'board_size': size,
            'square_size_m': square,
        }
    )
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(
        module, 'Node', lambda **kw: pytest.fail('Node created with invalid board')
    )
    with pytest.raises(ValueError):
        module._setup(context)


def test_board_units_are_meters_and_dimensions_are_inner_corners():
    assert board_settings('9x6', '0.024') == ((9, 6), 0.024)
