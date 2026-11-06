"""Regression tests for the released DRL-VO policy input contract."""

import numpy as np
import pytest

from nav2py_drl_vo_controller.planner import DrlVoInputError
from nav2py_drl_vo_controller.planner import DrlVoPlanner
from nav2py_drl_vo_controller.planner import decode_numeric_array
from nav2py_drl_vo_controller.planner import OBSERVATION_SIZE
from nav2py_drl_vo_controller.planner import PEDESTRIAN_MAP_SIZE
from nav2py_drl_vo_controller.planner import SCAN_HISTORY_FRAMES
from nav2py_drl_vo_controller.planner import SCAN_SAMPLES


class _FakeModel:
    def __init__(self, action=(0.0, 0.0)):
        self.action = np.asarray(action, dtype=np.float32)
        self.observation = None

    def predict(self, observation, deterministic):
        assert deterministic is True
        self.observation = observation
        return self.action, None


def _inputs(scan_value=15.0):
    return {
        'pedestrian_map': np.zeros(PEDESTRIAN_MAP_SIZE, dtype=np.float32),
        'scan_history': np.full(
            SCAN_HISTORY_FRAMES * SCAN_SAMPLES,
            scan_value,
            dtype=np.float32,
        ),
        'current_scan': np.full(SCAN_SAMPLES, scan_value, dtype=np.float32),
        'sub_goal': np.asarray([2.0, -2.0], dtype=np.float32),
    }


def test_observation_matches_released_policy_shape_and_scaling():
    model = _FakeModel()
    planner = DrlVoPlanner(model=model)
    data = _inputs()
    planner.compute_velocity_commands(**data)
    observation = model.observation
    assert observation.shape == (OBSERVATION_SIZE,)
    assert np.all(observation[:PEDESTRIAN_MAP_SIZE] == 0.0)
    assert np.allclose(
        observation[PEDESTRIAN_MAP_SIZE:-2], 0.0, atol=1e-6
    )
    assert np.array_equal(observation[-2:], np.asarray([1.0, -1.0]))


def test_action_is_scaled_to_forward_only_velocity_limits():
    planner = DrlVoPlanner(model=_FakeModel(action=(0.0, -0.5)))
    planner.configure(vx_limit=0.6, wz_limit=0.8)
    linear_x, angular_z = planner.compute_velocity_commands(**_inputs())
    assert linear_x == pytest.approx(0.3)
    assert angular_z == pytest.approx(-0.4)


def test_model_device_is_forwarded_to_checkpoint_loader(monkeypatch, tmp_path):
    checkpoint = tmp_path / 'policy.zip'
    checkpoint.write_bytes(b'test')
    loaded = {}

    def fake_load(model_file, model_device):
        loaded['path'] = model_file
        loaded['device'] = model_device
        return _FakeModel()

    monkeypatch.setattr(DrlVoPlanner, '_load_model', staticmethod(fake_load))
    DrlVoPlanner(model_file=checkpoint, model_device='cuda')
    assert loaded == {'path': checkpoint, 'device': 'cuda'}


def test_close_forward_obstacle_commands_a_stop_without_inference():
    model = _FakeModel(action=(1.0, 1.0))
    planner = DrlVoPlanner(model=model)
    data = _inputs()
    data['current_scan'][300] = 0.4
    assert planner.compute_velocity_commands(**data) == (0.0, 0.0)
    assert model.observation is None


def test_wrong_scan_history_size_fails_closed():
    planner = DrlVoPlanner(model=_FakeModel())
    data = _inputs()
    data['scan_history'] = data['scan_history'][:-1]
    with pytest.raises(DrlVoInputError, match='expected 7200'):
        planner.compute_velocity_commands(**data)


def test_compact_numeric_transport_round_trips_finite_values():
    import base64

    source = np.asarray([1.25, -2.0, 3.5], dtype='>f4')
    payload = b'b64f32be:' + base64.b64encode(source.tobytes())
    values = decode_numeric_array([payload], 3, 'test_values')
    assert np.array_equal(values, np.asarray([1.25, -2.0, 3.5]))


def test_compact_numeric_transport_rejects_wrong_size_and_nonfinite():
    import base64

    short = base64.b64encode(np.asarray([1.0, 2.0], dtype='>f4').tobytes())
    nonfinite = base64.b64encode(
        np.asarray([1.0, np.nan, 3.0], dtype='>f4').tobytes()
    )
    with pytest.raises(DrlVoInputError, match='expected 12'):
        decode_numeric_array([b'b64f32be:' + short], 3, 'test_values')
    with pytest.raises(DrlVoInputError, match='non-finite'):
        decode_numeric_array([b'b64f32be:' + nonfinite], 3, 'test_values')


def test_compact_numeric_transport_rejects_unknown_or_invalid_encoding():
    with pytest.raises(DrlVoInputError, match='unsupported encoding'):
        decode_numeric_array([b'1,2,3'], 3, 'test_values')
    with pytest.raises(DrlVoInputError, match='invalid base64'):
        decode_numeric_array([b'b64f32be:not_base64!'], 3, 'test_values')
