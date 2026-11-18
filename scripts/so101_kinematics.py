"""URDF forward kinematics and bounded, local XYZ inverse kinematics (NumPy only)."""

from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np


BODY_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "so101_new_calib.urdf"


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin_transform(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


@dataclass
class CartesianPlan:
    start: np.ndarray
    target_mm: np.ndarray
    waypoints: list
    xyz_mm: list
    hold_direction: bool


class SO101Kinematics:
    def __init__(self, path=MODEL_PATH, tool_offset_mm=(0, 0, 0)):
        self.tool_offset = np.asarray(tool_offset_mm, dtype=float)
        if self.tool_offset.shape != (3,) or not np.all(np.isfinite(self.tool_offset)) or np.linalg.norm(self.tool_offset) > 150:
            raise ValueError("Tool offset must be three finite values, no more than 150 mm long")
        root = ET.parse(path).getroot()
        by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        chain, link = [], "gripper_frame_link"
        while link != "base_link":
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        self.chain = []
        limits = {}
        for joint in reversed(chain):
            name = joint.get("name")
            moving = joint.get("type") != "fixed"
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ") if moving else None
            self.chain.append((name, origin_transform(joint.find("origin")), axis))
            if moving:
                limit = joint.find("limit")
                limits[name] = (math.degrees(float(limit.get("lower"))), math.degrees(float(limit.get("upper"))))
        if tuple(limits) != BODY_JOINTS:
            raise ValueError("Unexpected robot joint chain")
        self.limits = np.array([limits[joint] for joint in BODY_JOINTS])

    def transform(self, degrees):
        degrees = np.asarray(degrees, dtype=float)
        if degrees.shape != (5,) or not np.all(np.isfinite(degrees)):
            raise ValueError("Expected five finite body joint angles in degrees")
        angles = dict(zip(BODY_JOINTS, np.deg2rad(degrees)))
        result = np.eye(4)
        for name, origin, axis in self.chain:
            result = result @ origin
            if axis is not None:
                motion = np.eye(4)
                motion[:3, :3] = rotation(axis, angles[name])
                result = result @ motion
        result[:3, 3] += result[:3, :3] @ (self.tool_offset / 1000)
        return result

    def xyz(self, degrees):
        return self.transform(degrees)[:3, 3] * 1000

    def intersect_limits(self, calibrated):
        limits = self.limits.copy()
        for index, name in enumerate(BODY_JOINTS):
            low, high = calibrated[f"{name}.pos"]
            limits[index] = (max(low, limits[index, 0]), min(high, limits[index, 1]))
        if np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("Model/calibration joint limits do not overlap")
        return limits

    def solve(self, target_mm, seed_degrees, limits, direction=None, cancelled=lambda: False):
        target = np.asarray(target_mm, dtype=float)
        seed = np.asarray(seed_degrees, dtype=float)
        limits = np.asarray(limits, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)) or np.linalg.norm(target) > 1000:
            raise ValueError("Invalid or unreachable XYZ target")
        if limits.shape != (5, 2) or not np.all(np.isfinite(limits)) or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("Invalid joint limits")
        if seed.shape != (5,) or not np.all(np.isfinite(seed)) or np.any(seed < limits[:, 0]) or np.any(seed > limits[:, 1]):
            raise ValueError("Current pose is outside model/calibration limits; reposition with torque off")
        reference = np.deg2rad(seed)
        low, high = np.deg2rad(limits[:, 0]), np.deg2rad(limits[:, 1])
        q = reference.copy()
        if direction is not None:
            direction = np.asarray(direction, dtype=float)
            if direction.shape != (3,) or not np.all(np.isfinite(direction)) or np.linalg.norm(direction) < 1e-9:
                raise ValueError("Invalid tool direction")
            direction = direction / np.linalg.norm(direction)

        def residual(angles):
            transform = self.transform(np.rad2deg(angles))
            values = [(transform[:3, 3] * 1000 - target)]
            if direction is not None:
                values.append((transform[:3, 2] - direction) * 80)
            # Prefer a nearby solution rather than a different arm configuration.
            values.append((angles - reference) * 0.05)
            return np.concatenate(values)

        damping = 0.1
        for _ in range(120):
            if cancelled():
                raise RuntimeError("Planning cancelled")
            r = residual(q)
            jacobian = np.column_stack([(residual(q + np.eye(5)[i] * 1e-5) - r) / 1e-5 for i in range(5)])
            step = np.linalg.solve(jacobian.T @ jacobian + damping * np.eye(5), -jacobian.T @ r)
            step *= min(1.0, math.radians(2) / max(np.max(np.abs(step)), 1e-12))
            candidate = np.clip(q + step, low, high)
            if np.linalg.norm(residual(candidate)) < np.linalg.norm(r):
                q = candidate
                damping = max(0.001, damping * 0.5)
            else:
                damping = min(1e6, damping * 5)
            t = self.transform(np.rad2deg(q))
            error_mm = np.linalg.norm(t[:3, 3] * 1000 - target)
            angle_error = 0 if direction is None else math.degrees(math.acos(np.clip(t[:3, 2] @ direction, -1, 1)))
            if error_mm <= 0.15 and angle_error <= 0.5:
                return np.rad2deg(q)
        raise ValueError(f"No nearby IK solution within bounds (tip error {error_mm:.2f} mm, direction error {angle_error:.2f} degrees)")

    def plan(self, start, target_mm, limits, hold_direction=False, cancelled=lambda: False):
        start = np.asarray(start, dtype=float)
        target = np.asarray(target_mm, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("XYZ must contain three finite millimetre values")
        origin = self.xyz(start)
        distance = np.linalg.norm(target - origin)
        if distance > 10.000001:
            raise ValueError("This test permits at most 10 mm per preview/move. Start with a 1 mm jog.")
        steps = max(1, int(math.ceil(distance / 0.5)))
        direction = self.transform(start)[:3, 2] if hold_direction else None
        previous = start.copy()
        waypoints, points = [], []
        for fraction in np.linspace(0, 1, steps + 1)[1:]:
            point = origin + fraction * (target - origin)
            solved = self.solve(point, previous, limits, direction, cancelled)
            if np.max(np.abs(solved - previous)) > 2:
                raise ValueError("IK requires a joint change over 2 degrees between waypoints; try a smaller jog")
            if np.max(np.abs(solved - start)) > 10:
                raise ValueError("IK requires over 10 degrees of joint travel; try a different starting pose")
            waypoints.append(solved)
            points.append(point)
            previous = solved
        return CartesianPlan(start.copy(), target.copy(), waypoints, points, hold_direction)
