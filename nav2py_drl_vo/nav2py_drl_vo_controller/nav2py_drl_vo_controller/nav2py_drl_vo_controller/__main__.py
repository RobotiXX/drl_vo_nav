"""Socket process used by the DRL-VO Nav2 controller plugin."""

import argparse
import logging

import numpy as np
import yaml

import nav2py.interfaces

from .planner import DrlVoPlanner
from .planner import decode_numeric_array
from .planner import PEDESTRIAN_MAP_SIZE
from .planner import SCAN_HISTORY_FRAMES
from .planner import SCAN_SAMPLES


class DrlVoControllerProcess(nav2py.interfaces.nav2py_costmap_controller):
    """Run DRL-VO inference behind nav2py's loopback socket protocol."""

    def __init__(
        self,
        *args,
        model_file,
        range_limit,
        vx_limit,
        wz_limit,
        min_obstacle_distance,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.logger = logging.getLogger('nav2py_drl_vo_controller')
        self.planner = DrlVoPlanner(model_file=model_file)
        self.planner.configure(
            range_limit=range_limit,
            vx_limit=vx_limit,
            wz_limit=wz_limit,
            min_obstacle_distance=min_obstacle_distance,
        )
        warmup_seconds = self.planner.warm_up()
        self.current_scan = None
        self.scan_history = None
        self.pedestrian_map = np.zeros(PEDESTRIAN_MAP_SIZE, dtype=np.float32)
        self.sub_goal = None

        self._register_callback('scan', self._scan_callback)
        self._register_callback('scan_history', self._scan_history_callback)
        self._register_callback('pedestrian_map', self._pedestrian_map_callback)
        self._register_callback('data', self._data_callback)
        self.logger.info(
            'DRL-VO checkpoint loaded and warmed in %.3f s', warmup_seconds
        )

    @staticmethod
    def _decode_yaml(messages):
        if not isinstance(messages, list) or len(messages) != 1:
            raise ValueError('expected exactly one nav2py payload')
        payload = messages[0]
        if isinstance(payload, bytes):
            payload = payload.decode('utf-8')
        return yaml.safe_load(payload)

    def _scan_callback(self, messages):
        self.current_scan = decode_numeric_array(
            messages, SCAN_SAMPLES, 'current_scan'
        )

    def _scan_history_callback(self, messages):
        self.scan_history = decode_numeric_array(
            messages,
            SCAN_HISTORY_FRAMES * SCAN_SAMPLES,
            'scan_history',
        )

    def _pedestrian_map_callback(self, messages):
        self.pedestrian_map = decode_numeric_array(
            messages, PEDESTRIAN_MAP_SIZE, 'pedestrian_map'
        )

    def _data_callback(self, messages):
        """Update the subgoal, infer once, and answer the waiting Nav2 tick."""
        try:
            data = self._decode_yaml(messages)
            goal = data['goal_pose']['position']
            self.sub_goal = np.asarray([goal['x'], goal['y']], dtype=np.float32)
            if self.current_scan is None:
                raise ValueError('current scan is unavailable')
            if self.scan_history is None:
                raise ValueError('scan history is unavailable')
            if self.scan_history.size != SCAN_HISTORY_FRAMES * SCAN_SAMPLES:
                raise ValueError('scan history is not ready')
            linear_x, angular_z = self.planner.compute_velocity_commands(
                pedestrian_map=self.pedestrian_map,
                scan_history=self.scan_history,
                current_scan=self.current_scan,
                sub_goal=self.sub_goal,
            )
            self._send_cmd_vel(linear_x, angular_z)
        except Exception:
            self.logger.exception('DRL-VO inference failed; commanding stop')
            self._send_cmd_vel(0.0, 0.0)


def build_argument_parser():
    """Create the inference-process CLI parser."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--model-file', required=True)
    parser.add_argument('--range-limit', type=float, default=30.0)
    parser.add_argument('--vx-limit', type=float, default=0.5)
    parser.add_argument('--wz-limit', type=float, default=0.7)
    parser.add_argument('--min-obstacle-distance', type=float, default=0.45)
    return parser


def main(argv=None):
    """Start the loopback inference server."""
    logging.basicConfig(level=logging.INFO)
    arguments = build_argument_parser().parse_args(argv)
    controller = DrlVoControllerProcess(
        host=arguments.host,
        port=arguments.port,
        model_file=arguments.model_file,
        range_limit=arguments.range_limit,
        vx_limit=arguments.vx_limit,
        wz_limit=arguments.wz_limit,
        min_obstacle_distance=arguments.min_obstacle_distance,
    )
    try:
        controller.serve()
    except (ConnectionError, KeyboardInterrupt):
        pass
    finally:
        controller.cleanup()


if __name__ == '__main__':
    main()
