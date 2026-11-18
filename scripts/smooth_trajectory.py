"""Continuous, non-overshooting joint interpolation with speed-scaled timing."""

import numpy as np


class SmoothJointTrajectory:
    def __init__(self, joints, xyz_mm, joint_speed, tip_speed, tip_xyz=None, cancelled=lambda: False):
        self.joints = np.asarray(joints, dtype=float)
        xyz = np.asarray(xyz_mm, dtype=float)
        if (self.joints.ndim != 2 or len(self.joints) < 2 or not np.all(np.isfinite(self.joints))
                or xyz.shape != (len(self.joints), 3) or not np.all(np.isfinite(xyz))):
            raise ValueError("A trajectory needs finite joint and XYZ waypoint arrays")
        if not np.isfinite(joint_speed) or not np.isfinite(tip_speed) or min(joint_speed, tip_speed) <= 0:
            raise ValueError("Trajectory speeds must be finite and positive")
        durations = np.maximum.reduce([
            np.linalg.norm(np.diff(xyz, axis=0), axis=1) / tip_speed,
            np.max(np.abs(np.diff(self.joints, axis=0)), axis=1) / joint_speed,
            np.full(len(self.joints) - 1, 0.05),
        ])
        self.times = np.r_[0., np.cumsum(durations)]
        secants = np.diff(self.joints, axis=0) / durations[:, None]
        self.slopes = np.zeros_like(self.joints)  # Rest only at whole-path endpoints.
        for index in range(1, len(self.joints) - 1):
            before, after = secants[index - 1], secants[index]
            same_direction = before * after > 0
            w1 = 2 * durations[index] + durations[index - 1]
            w2 = durations[index] + 2 * durations[index - 1]
            self.slopes[index, same_direction] = (w1 + w2) / (
                w1 / before[same_direction] + w2 / after[same_direction])

        ratio = 1.
        # Joint velocity is quadratic within each segment. Check its exact extrema.
        for index, duration in enumerate(durations):
            if cancelled():
                raise RuntimeError("Planning cancelled")
            a, b, c, _ = self.coefficients(index)
            extreme = -b / np.where(np.abs(a) > 1e-12, 3 * a, 1.)
            speed = np.maximum(np.abs(self.slopes[index]), np.abs(self.slopes[index + 1]))
            valid = (np.abs(a) > 1e-12) & (extreme > 0) & (extreme < 1)
            speed = np.maximum(speed, np.where(valid, np.abs((3 * a * extreme ** 2 + 2 * b * extreme + c) / duration), 0))
            ratio = max(ratio, float(np.max(speed)) / joint_speed)
            if tip_xyz is not None:
                for fraction in np.linspace(0, 1, 9):
                    point, velocity = self.sample(self.times[index] + fraction * duration)
                    # Model-tip speed estimate along the smooth curve, not just its chord.
                    tip_velocity = (tip_xyz(point + velocity * 1e-3) - tip_xyz(point - velocity * 1e-3)) / 2e-3
                    ratio = max(ratio, float(np.linalg.norm(tip_velocity)) / tip_speed)
        scale = ratio * 1.02
        self.times *= scale
        self.slopes /= scale

    @property
    def duration(self):
        return float(self.times[-1])

    def coefficients(self, index):
        y0, y1 = self.joints[index:index + 2]
        duration = self.times[index + 1] - self.times[index]
        m0, m1 = self.slopes[index:index + 2]
        return (2 * y0 - 2 * y1 + duration * (m0 + m1),
                -3 * y0 + 3 * y1 - duration * (2 * m0 + m1), duration * m0, y0)

    def sample(self, elapsed):
        elapsed = float(np.clip(elapsed, 0, self.duration))
        index = min(len(self.joints) - 2, max(0, int(np.searchsorted(self.times, elapsed, side="right") - 1)))
        duration = self.times[index + 1] - self.times[index]
        s = (elapsed - self.times[index]) / duration
        a, b, c, d = self.coefficients(index)
        return ((a * s ** 3 + b * s ** 2 + c * s + d),
                (3 * a * s ** 2 + 2 * b * s + c) / duration)
