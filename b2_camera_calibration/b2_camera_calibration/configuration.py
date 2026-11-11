"""Measured checkerboard inputs and physical camera identities."""

import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory


def default_output_root():
    """Save measured camera files with the owning driver, including symlink installs."""
    directory = Path(get_package_share_directory('b2_camera')) / 'config/calibration'
    return str((directory / 'README.md').resolve().parent)


def board_settings(size, square):
    match = re.fullmatch(r'([0-9]+)x([0-9]+)', size.strip())
    if match is None or min(map(int, match.groups())) < 3:
        raise ValueError('board_size must be NxM inner corners, with each dimension >=3')
    try:
        square = float(square)
    except (TypeError, ValueError) as exc:
        raise ValueError('square_size_m must be a measured length in meters') from exc
    if not math.isfinite(square) or square <= 0:
        raise ValueError('square_size_m must be finite and greater than zero')
    return tuple(map(int, match.groups())), square


def camera_identity(side):
    if side not in ('front', 'back'):
        raise ValueError('camera_side must be front or back')
    return f'b2_{side}_camera', f'/b2/{side}_camera'


def output_directory(value):
    if not value.strip():
        raise ValueError('output_root must name the calibration results directory')
    return Path(value).expanduser().resolve()
