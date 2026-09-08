"""Training-compatible DRL-VO observation preprocessing and inference."""

import base64
import binascii
from pathlib import Path
import sys
import time

import numpy as np


PEDESTRIAN_MAP_SIZE = 2 * 80 * 80
SCAN_HISTORY_FRAMES = 10
SCAN_SAMPLES = 720
SCAN_FEATURE_SIZE = 20 * 80 * 4
OBSERVATION_SIZE = PEDESTRIAN_MAP_SIZE + SCAN_FEATURE_SIZE + 2


class DrlVoInputError(ValueError):
    """Raised when runtime sensor data does not match the trained policy."""


def decode_numeric_array(messages, expected_size, name):
    """Decode one base64, big-endian float32 nav2py array payload."""
    if not isinstance(messages, list) or len(messages) != 1:
        raise DrlVoInputError(f'{name} expected exactly one nav2py payload')
    payload = messages[0]
    prefix = b'b64f32be:'
    if isinstance(payload, str):
        payload = payload.encode('ascii')
    if not isinstance(payload, bytes) or not payload.startswith(prefix):
        raise DrlVoInputError(f'{name} has an unsupported encoding')
    try:
        decoded = base64.b64decode(payload[len(prefix):], validate=True)
    except (binascii.Error, ValueError) as error:
        raise DrlVoInputError(f'{name} has invalid base64 data') from error
    expected_bytes = expected_size * np.dtype(np.float32).itemsize
    if len(decoded) != expected_bytes:
        raise DrlVoInputError(
            f'{name} has {len(decoded)} bytes; expected {expected_bytes}'
        )
    values = np.frombuffer(decoded, dtype='>f4').astype(np.float32)
    if not np.all(np.isfinite(values)):
        raise DrlVoInputError(f'{name} contains a non-finite value')
    return values


class DrlVoPlanner:
    """Load the DRL-VO policy and convert ROS-facing arrays into observations."""

    def __init__(self, model_file=None, model=None):
        if model is not None:
            self.model = model
        else:
            if not model_file:
                raise DrlVoInputError('model_file is required')
            self.model = self._load_model(Path(model_file))
        self.range_limit = 30.0
        self.vx_limit = 0.5
        self.wz_limit = 0.7
        self.min_obstacle_distance = 0.45

    @staticmethod
    def _load_model(model_file):
        if not model_file.is_file():
            raise DrlVoInputError(f'DRL-VO checkpoint does not exist: {model_file}')

        # The Stable-Baselines3 archive stores this historical top-level module
        # name. Register the packaged module before unpickling the trusted model.
        from . import custom_cnn_full
        sys.modules.setdefault('custom_cnn_full', custom_cnn_full)
        from stable_baselines3 import PPO

        return PPO.load(str(model_file), device='cpu')

    def configure(
        self,
        range_limit=30.0,
        vx_limit=0.5,
        wz_limit=0.7,
        min_obstacle_distance=0.45,
    ):
        """Configure physical scaling applied around the fixed policy."""
        if range_limit <= 0.0:
            raise DrlVoInputError('range_limit must be positive')
        if vx_limit <= 0.0 or wz_limit <= 0.0:
            raise DrlVoInputError('velocity limits must be positive')
        if min_obstacle_distance < 0.0:
            raise DrlVoInputError('min_obstacle_distance cannot be negative')
        self.range_limit = float(range_limit)
        self.vx_limit = float(vx_limit)
        self.wz_limit = float(wz_limit)
        self.min_obstacle_distance = float(min_obstacle_distance)

    def warm_up(self):
        """Run one discarded inference before the real-time control loop."""
        start = time.perf_counter()
        self.compute_velocity_commands(
            pedestrian_map=np.zeros(PEDESTRIAN_MAP_SIZE, dtype=np.float32),
            scan_history=np.full(
                SCAN_HISTORY_FRAMES * SCAN_SAMPLES,
                self.range_limit,
                dtype=np.float32,
            ),
            current_scan=np.full(
                SCAN_SAMPLES, self.range_limit, dtype=np.float32
            ),
            sub_goal=np.asarray([2.0, 0.0], dtype=np.float32),
        )
        return time.perf_counter() - start

    @staticmethod
    def _finite_array(values, expected_size, name):
        result = np.asarray(values, dtype=np.float32).reshape(-1)
        if result.size != expected_size:
            raise DrlVoInputError(
                f'{name} has {result.size} values; expected {expected_size}'
            )
        if not np.all(np.isfinite(result)):
            raise DrlVoInputError(f'{name} contains a non-finite value')
        return result

    def build_observation(self, pedestrian_map, scan_history, sub_goal):
        """Build the 19,202-value observation used by the released model."""
        pedestrians = self._finite_array(
            pedestrian_map, PEDESTRIAN_MAP_SIZE, 'pedestrian_map'
        )
        scans = self._finite_array(
            scan_history,
            SCAN_HISTORY_FRAMES * SCAN_SAMPLES,
            'scan_history',
        ).reshape(SCAN_HISTORY_FRAMES, 80, 9)
        goal = self._finite_array(sub_goal, 2, 'sub_goal')

        # These transformations match the ROS 1 inference implementation used
        # to train the released archive: velocities and the 2 m subgoal are
        # scaled from [-2, 2] to [-1, 1].
        pedestrian_features = np.clip(pedestrians, -2.0, 2.0) / 2.0
        goal_features = np.clip(goal, -2.0, 2.0) / 2.0

        scan_min = np.min(scans, axis=2)
        scan_mean = np.mean(scans, axis=2)
        scan_features = np.empty((SCAN_HISTORY_FRAMES, 2, 80), dtype=np.float32)
        scan_features[:, 0, :] = scan_min
        scan_features[:, 1, :] = scan_mean
        scan_features = np.tile(scan_features.reshape(-1), 4)
        scan_features = np.clip(scan_features, 0.0, self.range_limit)
        scan_features = 2.0 * scan_features / self.range_limit - 1.0

        observation = np.concatenate(
            (pedestrian_features, scan_features, goal_features)
        ).astype(np.float32, copy=False)
        if observation.size != OBSERVATION_SIZE:
            raise AssertionError('internal DRL-VO observation shape mismatch')
        return observation

    def compute_velocity_commands(
        self,
        pedestrian_map,
        scan_history,
        current_scan,
        sub_goal,
    ):
        """Return forward and angular velocity for one synchronized sample."""
        scan = self._finite_array(current_scan, SCAN_SAMPLES, 'current_scan')

        # The original 1080-ray lidar was cropped to a 240-degree, 720-value
        # model scan. Indices 180:540 are therefore the forward +/-60 degrees.
        forward_scan = scan[180:540]
        valid_forward_scan = forward_scan[forward_scan > 0.0]
        if (
            valid_forward_scan.size
            and np.min(valid_forward_scan) <= self.min_obstacle_distance
        ):
            return 0.0, 0.0

        observation = self.build_observation(
            pedestrian_map, scan_history, sub_goal
        )
        action, _state = self.model.predict(observation, deterministic=True)
        action = self._finite_array(action, 2, 'policy_action')
        action = np.clip(action, -1.0, 1.0)

        linear_x = (float(action[0]) + 1.0) * self.vx_limit / 2.0
        angular_z = float(action[1]) * self.wz_limit
        return linear_x, angular_z
