"""Compose DRL-VO into a rendered RWD Nav2 parameter profile."""

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import yaml


class ProfileError(ValueError):
    """Raised when a Nav2 profile cannot be composed safely."""


class _NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        """Materialize values because ROS 2 parameters reject YAML aliases."""
        return True


def _load_mapping(path):
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ProfileError(f'Unable to load YAML file: {path}') from error
    if not isinstance(data, dict):
        raise ProfileError(f'YAML root must be a mapping: {path}')
    return data


def _controller_parameters(config):
    try:
        return config['controller_server']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ProfileError(
            "Nav2 YAML has no controller_server.ros__parameters mapping"
        ) from error


def compose_profile(base, controller, mode, model_file):
    """Return a copy of the base profile with FollowPath replaced by DRL-VO."""
    if mode not in {'social', 'lidar_ablation'}:
        raise ProfileError(f"Unsupported DRL-VO mode '{mode}'")
    output = deepcopy(base)
    base_parameters = _controller_parameters(output)
    plugins = base_parameters.get('controller_plugins')
    if not isinstance(plugins, list) or 'FollowPath' not in plugins:
        raise ProfileError("controller_plugins must contain 'FollowPath'")
    template_parameters = _controller_parameters(controller)
    replacement = deepcopy(template_parameters.get('FollowPath'))
    if not isinstance(replacement, dict):
        raise ProfileError('Controller YAML has no FollowPath mapping')
    replacement['model_file'] = str(Path(model_file).expanduser().resolve())
    replacement['require_pedestrian_data'] = mode == 'social'
    base_parameters['FollowPath'] = replacement
    return output


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write_profile(path, profile):
    """Write a non-overwriting, ROS-compatible parameter file."""
    destination = Path(path).expanduser().resolve()
    if destination.exists():
        raise ProfileError(f'Output already exists: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.dump(profile, Dumper=_NoAliasDumper, sort_keys=False),
        encoding='utf-8',
    )
    return destination


def _describe_file(path, label):
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise ProfileError(f'{label} does not exist: {resolved}')
    return {
        'path': str(resolved),
        'sha256': _sha256(resolved),
    }


def write_context(
    path,
    mode,
    model_file,
    controller_file,
    detector_file=None,
    reid_file=None,
    detector_device=None,
    pedestrian_source=None,
    perception_config=None,
):
    """Write immutable task-associated metadata for an experiment run."""
    destination = Path(path).expanduser().resolve()
    if destination.exists():
        raise ProfileError(f'Context output already exists: {destination}')
    context = {
        'navigation_profile': f'drl_vo_{mode}',
        'global_planner': 'theta_star',
        'local_controller': 'drl_vo',
        'pedestrian_input': (
            '/drl_vo/pedestrian_velocity_map'
            if mode == 'social' else 'disabled_zero_map'
        ),
        'pedestrian_source': pedestrian_source or (
            'external_velocity_map'
            if mode == 'social' else 'internal_zero_map'
        ),
        'checkpoint': _describe_file(model_file, 'DRL-VO checkpoint'),
        'controller_config': _describe_file(
            controller_file, 'DRL-VO controller configuration'
        ),
    }
    if detector_file is not None:
        context['social_perception'] = {
            'detector_model': _describe_file(
                detector_file, 'Detector model'
            ),
            'reid_model': (
                _describe_file(reid_file, 'ReID model')
                if reid_file is not None else None
            ),
            'device': str(detector_device or ''),
        }
    if perception_config is not None:
        context.setdefault('social_perception', {})[
            'configuration'
        ] = _describe_file(perception_config, 'Perception configuration')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(context, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return destination


def build_argument_parser():
    """Create the profile-composition CLI parser."""
    parser = argparse.ArgumentParser()
    parser.add_argument('base', type=Path, help='Rendered RWD Nav2 YAML.')
    parser.add_argument('--controller-config', required=True, type=Path)
    parser.add_argument('--model-file', required=True, type=Path)
    parser.add_argument('--mode', choices=('social', 'lidar_ablation'), required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--context-output', type=Path)
    parser.add_argument('--detector-model', type=Path)
    parser.add_argument('--reid-model', type=Path)
    parser.add_argument('--detector-device')
    parser.add_argument('--pedestrian-source')
    parser.add_argument('--perception-config', type=Path)
    return parser


def main(argv=None):
    """Compose and write one DRL-VO RWD experiment profile."""
    arguments = build_argument_parser().parse_args(argv)
    profile = compose_profile(
        _load_mapping(arguments.base),
        _load_mapping(arguments.controller_config),
        arguments.mode,
        arguments.model_file,
    )
    output = write_profile(arguments.output, profile)
    print(f'Wrote DRL-VO {arguments.mode} Nav2 profile to {output}')
    if arguments.context_output:
        context = write_context(
            arguments.context_output,
            arguments.mode,
            arguments.model_file,
            arguments.controller_config,
            arguments.detector_model,
            arguments.reid_model,
            arguments.detector_device,
            arguments.pedestrian_source,
            arguments.perception_config,
        )
        print(f'Wrote DRL-VO experiment context to {context}')


if __name__ == '__main__':
    main()
