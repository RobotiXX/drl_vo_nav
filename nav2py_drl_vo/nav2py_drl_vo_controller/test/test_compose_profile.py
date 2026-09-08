"""Tests for non-destructive RWD Nav2 profile composition."""

import json

from nav2py_drl_vo_controller.compose_profile import compose_profile
from nav2py_drl_vo_controller.compose_profile import write_context


def _base():
    return {
        'controller_server': {
            'ros__parameters': {
                'controller_plugins': ['FollowPath', 'DoorTraversal'],
                'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner'},
                'DoorTraversal': {'plugin': 'dwb_core::DWBLocalPlanner'},
            }
        }
    }


def _controller():
    return {
        'controller_server': {
            'ros__parameters': {
                'FollowPath': {
                    'plugin': 'nav2_rotation_shim_controller::RotationShimController',
                    'primary_controller': (
                        'nav2py_drl_vo_controller::DrlVoController'
                    ),
                    'require_pedestrian_data': True,
                }
            }
        }
    }


def test_social_mode_requires_pedestrian_input_and_preserves_other_plugins(tmp_path):
    base = _base()
    output = compose_profile(base, _controller(), 'social', tmp_path / 'model.zip')
    parameters = output['controller_server']['ros__parameters']
    assert parameters['FollowPath']['require_pedestrian_data'] is True
    assert parameters['DoorTraversal'] == base['controller_server'][
        'ros__parameters'
    ]['DoorTraversal']
    assert output is not base


def test_lidar_ablation_is_explicitly_marked_by_disabled_pedestrian_gate(tmp_path):
    output = compose_profile(
        _base(), _controller(), 'lidar_ablation', tmp_path / 'model.zip'
    )
    follow_path = output['controller_server']['ros__parameters']['FollowPath']
    assert follow_path['require_pedestrian_data'] is False


def test_study_context_hashes_social_perception_models(tmp_path):
    policy = tmp_path / 'policy.zip'
    controller = tmp_path / 'controller.yaml'
    detector = tmp_path / 'detector.engine'
    reid = tmp_path / 'reid.pt'
    policy.write_bytes(b'policy')
    controller.write_text('controller: drl_vo\n', encoding='utf-8')
    detector.write_bytes(b'detector')
    reid.write_bytes(b'reid')

    output = write_context(
        tmp_path / 'context.json',
        'social',
        policy,
        controller,
        detector,
        reid,
        '0',
        'hardware_camera_lidar_tracking',
    )
    context = json.loads(output.read_text(encoding='utf-8'))

    assert context['social_perception']['detector_model']['path'] == str(
        detector.resolve()
    )
    assert len(
        context['social_perception']['detector_model']['sha256']
    ) == 64
    assert context['social_perception']['reid_model']['path'] == str(
        reid.resolve()
    )
    assert context['social_perception']['device'] == '0'
    assert context['pedestrian_source'] == 'hardware_camera_lidar_tracking'


def test_study_context_hashes_native_perception_configuration(tmp_path):
    policy = tmp_path / 'policy.zip'
    controller = tmp_path / 'controller.yaml'
    perception = tmp_path / 'zed.yaml'
    policy.write_bytes(b'policy')
    controller.write_text('controller: drl_vo\n', encoding='utf-8')
    perception.write_text('zed object detection\n', encoding='utf-8')

    output = write_context(
        tmp_path / 'context.json',
        'social',
        policy,
        controller,
        pedestrian_source='zed2_sdk_object_detection_tracking',
        perception_config=perception,
    )
    context = json.loads(output.read_text(encoding='utf-8'))

    assert context['pedestrian_source'] == (
        'zed2_sdk_object_detection_tracking'
    )
    config = context['social_perception']['configuration']
    assert config['path'] == str(perception.resolve())
    assert len(config['sha256']) == 64
