"""Geometry and timestamp selection for a virtual obstacle scan, without ROS."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScanGrid:
    """A circular scan with one bin per direction (no duplicated seam)."""

    bins: int = 360
    range_min: float = 0.05
    range_max: float = 12.0

    def __post_init__(self):
        if not isinstance(self.bins, int) or self.bins < 4:
            raise ValueError('bins must be an integer >= 4')
        if not (math.isfinite(self.range_min) and math.isfinite(self.range_max)
                and 0 <= self.range_min < self.range_max):
            raise ValueError('ranges must be finite, with 0 <= range_min < range_max')

    @property
    def angle_min(self):
        """Angle of the first bin, in radians."""
        return -math.pi

    @property
    def angle_increment(self):
        """Angular spacing, in radians."""
        return math.tau / self.bins

    @property
    def angle_max(self):
        """Angle of the last bin, consistent with the range array length."""
        return self.angle_min + (self.bins - 1) * self.angle_increment


@dataclass(frozen=True)
class ScanSample:
    """A sensor scan snapshot; its ranges are in its own frame."""

    stamp_ns: int
    frame_id: str
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray

    def __post_init__(self):
        if not self.frame_id:
            raise ValueError('scan frame_id is empty')
        if not (math.isfinite(self.angle_min) and math.isfinite(self.angle_increment)
                and self.angle_increment != 0):
            raise ValueError('scan angles must be finite with a nonzero increment')
        if not (math.isfinite(self.range_min) and math.isfinite(self.range_max)
                and 0 <= self.range_min < self.range_max):
            raise ValueError('invalid input scan range limits')
        if self.ranges.ndim != 1:
            raise ValueError('scan ranges must be one-dimensional')

    def points(self):
        """Return only finite, in-range hit endpoints as Nx3 points."""
        valid = (np.isfinite(self.ranges) & (self.ranges > 0)
                 & (self.ranges >= self.range_min) & (self.ranges <= self.range_max))
        distances = self.ranges[valid]
        angles = self.angle_min + np.flatnonzero(valid) * self.angle_increment
        return np.column_stack((distances * np.cos(angles),
                                distances * np.sin(angles), np.zeros(len(distances))))


def is_fresh(stamp_ns, now_ns, max_age_ns, future_tolerance_ns):
    """Reject zero stamps (TF's latest sentinel), old data and clock mismatch."""
    return stamp_ns > 0 and -future_tolerance_ns <= now_ns - stamp_ns <= max_age_ns


def closest_sample(samples, reference_ns, now_ns, max_age_ns, tolerance_ns):
    """Select the nearest fresh scan within the allowed timestamp difference."""
    candidates = [sample for sample in samples
                  if is_fresh(sample.stamp_ns, now_ns, max_age_ns, tolerance_ns)
                  and abs(sample.stamp_ns - reference_ns) <= tolerance_ns]
    return min(candidates, key=lambda s: (abs(s.stamp_ns - reference_ns), s.stamp_ns),
               default=None)


def project_hits(sample, matrix, grid):
    """Transform endpoints before binning; never infer free rays from endpoints."""
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('transform must be a finite 4x4 matrix')
    points = sample.points()
    points = points @ matrix[:3, :3].T + matrix[:3, 3]
    valid = np.isfinite(points).all(axis=1)
    xy = points[valid, :2]
    distances = np.hypot(xy[:, 0], xy[:, 1])
    valid = ((distances >= grid.range_min) & (distances <= grid.range_max)
             & (distances > 0))
    xy, distances = xy[valid], distances[valid]
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    # Nearest bin, with both +pi and -pi assigned to the circular seam.
    bins = (np.floor((angles - grid.angle_min) / grid.angle_increment + 0.5)
            .astype(np.int64) % grid.bins)
    ranges = np.full(grid.bins, np.inf, dtype=np.float64)
    np.minimum.at(ranges, bins, distances)
    ranges[~np.isfinite(ranges)] = np.nan
    return ranges.astype(np.float32)


def fuse_hits(lidar, lidar_matrix, camera, camera_matrix, grid):
    """Keep the nearest endpoint per bin; unknown bins stay NaN, never +inf."""
    ranges = project_hits(lidar, lidar_matrix, grid)
    if camera is not None:
        ranges = np.fmin(ranges, project_hits(camera, camera_matrix, grid))
    return ranges
