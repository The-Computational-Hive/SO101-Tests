"""Separate SO101 XYZ jogging test. --demo never opens a serial port."""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from lerobot.robots.so_follower import SO101FollowerConfig

try:
    from .joint_sliders import ArmController, DemoRobot, JOINTS, calibrated_limits, disable_all_torque
    from .so101_kinematics import BODY_JOINTS, MODEL_PATH, SO101Kinematics
    from .test_api import SafeStartFollower
    from .smooth_trajectory import SmoothJointTrajectory
except ImportError:
    from joint_sliders import ArmController, DemoRobot, JOINTS, calibrated_limits, disable_all_torque
    from so101_kinematics import BODY_JOINTS, MODEL_PATH, SO101Kinematics
    from test_api import SafeStartFollower
    from smooth_trajectory import SmoothJointTrajectory


def body_vector(pose):
    return np.array([pose[f"{joint}.pos"] for joint in BODY_JOINTS])


def full_pose(body, reference):
    pose = reference.copy()  # In particular, never move the gripper opening.
    pose.update({f"{joint}.pos": float(value) for joint, value in zip(BODY_JOINTS, body)})
    return pose


class CartesianWorker(threading.Thread):
    """Serial I/O, planning, and execution share one thread."""

    def __init__(self, args):
        super().__init__(daemon=False)
        self.args = args
        self.kinematics = SO101Kinematics(args.model, args.tool_offset_mm)
        self.requests, self.updates = queue.Queue(), queue.Queue()
        self.stop_event, self.close_event = threading.Event(), threading.Event()
        self.heartbeat = time.monotonic()
        self.controller = self.robot = None
        self.pending = self.motion = None
        self.faulted = False
        self.emergency_lockout = False
        self.sequence = 0
        self.log = self.writer = None

    def emit(self, kind, value=None):
        self.updates.put((kind, value))

    def cancelled(self):
        return self.close_event.is_set() or self.stop_event.is_set() or time.monotonic() - self.heartbeat > 0.75

    def state(self):
        self.emit("state", {
            "connected": self.controller is not None,
            "enabled": bool(self.controller and self.controller.enabled),
            "locked": self.emergency_lockout or bool(self.controller and self.controller.shutdown_failed),
            "moving": self.motion is not None,
        })

    def stop(self):
        self.motion = self.pending = None
        while not self.requests.empty():
            self.requests.get_nowait()
        try:
            if self.controller:
                self.controller.stop()
            elif self.robot and self.robot.bus.is_connected:
                disable_all_torque(self.robot.bus)
        except Exception:
            self.emergency_lockout = True
            self.state()
            self.emit("invalidate")
            raise
        self.state()
        self.emit("invalidate")

    def connect(self):
        if self.emergency_lockout:
            raise RuntimeError("Unconfirmed shutdown: disconnect motor power, inspect connections, and restart this app")
        if self.controller:
            return
        if self.args.demo:
            self.robot = DemoRobot()
            self.robot.positions.update(full_pose([0, -30, 30, 30, -90], self.robot.positions))
            self.robot.goals = self.robot.positions.copy()
            limits = {f"{joint}.pos": (0, 100) if joint == "gripper" else tuple(self.kinematics.limits[BODY_JOINTS.index(joint)]) for joint in JOINTS}
        else:
            self.robot = SafeStartFollower(SO101FollowerConfig(
                port=self.args.port, id=self.args.robot_id, use_degrees=True,
                max_relative_target={joint: 3.5 if joint in ("shoulder_lift", "elbow_flex") else 3.0 for joint in JOINTS},
            ))
            if not self.robot.calibration:
                raise RuntimeError("No saved calibration found for this robot ID")
            self.robot.connect(calibrate=False)
            if not self.robot.is_calibrated:
                raise RuntimeError("Saved calibration does not match motors")
            limits = calibrated_limits(self.robot.calibration)
        self.limits = self.kinematics.intersect_limits(limits)
        self.controller = ArmController(
            self.robot, limits, {"shoulder_lift": self.args.shoulder_p, "elbow_flex": self.args.elbow_p},
            {"shoulder_lift.pos": 3.0, "elbow_flex.pos": 3.0}, settle_tolerance=2.0,
        )
        Path("logs").mkdir(exist_ok=True)
        path = Path("logs") / f"xyz_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        self.log = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log)
        self.writer.writerow(["time_monotonic", "phase", "joint", "commanded_deg_or_units", "actual_deg_or_units",
                              "estimated_x_mm", "estimated_y_mm", "estimated_z_mm"])
        print(f"XYZ diagnostic log: {path.resolve()}", flush=True)
        self.faulted = False
        self.state()
        self.emit("status", "Connected with torque off. Inspect the estimated XYZ and begin with a 1 mm jog.")
        self.emit("target", self.kinematics.xyz(body_vector(self.controller.pose())))

    def preview(self, target, hold_direction, revision):
        if not self.controller or self.motion:
            raise ValueError("Connect first and finish/cancel the current move")
        actual = self.controller.pose()
        reference = self.controller.commanded.copy() if self.controller.enabled else actual.copy()
        plan = self.kinematics.plan(body_vector(reference), target, self.limits, hold_direction, self.cancelled)
        self.sequence += 1
        self.pending = (self.sequence, plan, reference, actual)
        self.emit("preview", {"id": self.sequence, "revision": revision, "target": plan.target_mm,
                              "points": [self.kinematics.xyz(plan.start), *plan.xyz_mm],
                              "start": plan.start, "end": plan.waypoints[-1]})
        self.emit("status", "Preview ready; no motion sent. Move to XYZ validates the current input and moves.")

    def start_motion(self, identifier):
        if not self.controller or not self.controller.enabled or self.pending is None or self.pending[0] != identifier:
            raise ValueError("Enable holding, then generate a fresh preview")
        _, plan, reference, preview_actual = self.pending
        actual = self.controller.pose()
        if any(abs(actual[key] - preview_actual[key]) > 1.0 for key in actual):
            self.pending = None
            self.emit("invalidate")
            raise ValueError("Arm drifted since preview; capture/preview again")
        if any(abs(self.controller.commanded[key] - reference[key]) > 1e-6 for key in reference):
            raise ValueError("Held target changed since preview; preview again")
        trajectory = SmoothJointTrajectory(
            [plan.start, *plan.waypoints], [self.kinematics.xyz(plan.start), *plan.xyz_mm],
            self.args.joint_speed, self.args.tip_speed, self.kinematics.xyz, self.cancelled,
        )
        # Smoothing must not silently turn the validated line into a large curve.
        origin = self.kinematics.xyz(plan.start)
        delta = plan.target_mm - origin
        for stamp in np.linspace(0, trajectory.duration, max(3, len(plan.waypoints) * 9)):
            if self.cancelled():
                raise RuntimeError("Planning cancelled")
            angles, _ = trajectory.sample(stamp)
            tip = self.kinematics.transform(angles)
            fraction = np.clip((tip[:3, 3] * 1000 - origin) @ delta / max(delta @ delta, 1e-12), 0, 1)
            if np.linalg.norm(tip[:3, 3] * 1000 - (origin + fraction * delta)) > 0.3:
                raise ValueError("Smoothed path deviates over 0.3 mm from the model line; try a smaller jog")
            if plan.hold_direction and tip[:3, 2] @ self.kinematics.transform(plan.start)[:3, 2] < np.cos(np.deg2rad(1)):
                raise ValueError("Smoothed path changes tool direction too much; try a smaller jog")
        self.motion = {"plan": plan, "reference": reference, "trajectory": trajectory, "elapsed": 0.,
                       "last_requested": reference.copy()}
        self.pending = None
        self.emit("invalidate")
        self.emit("status", "Moving along one smooth validated path. Model path is not collision checked.")
        self.state()

    def desired(self, dt, now):
        if self.motion is None:
            return self.controller.commanded.copy()
        motion = self.motion
        plan, trajectory = motion["plan"], motion["trajectory"]
        # Freeze path advancement if the existing joint controller cannot keep up.
        caught_up = all(abs(self.controller.commanded[key] - motion["last_requested"][key]) <= 0.1 for key in self.controller.commanded)
        if caught_up:
            motion["elapsed"] = min(trajectory.duration, motion["elapsed"] + min(dt, 0.1))
        angles, _ = trajectory.sample(motion["elapsed"])
        requested = full_pose(angles, motion["reference"])
        motion["last_requested"] = requested
        final_command_sent = all(abs(self.controller.commanded[key] - requested[key]) < 1e-6 for key in requested)
        if motion["elapsed"] >= trajectory.duration and final_command_sent:
            actual_xyz = self.kinematics.xyz(body_vector(self.controller.pose()))
            error = np.linalg.norm(actual_xyz - plan.target_mm)
            self.motion = None
            self.emit("status", f"Command finished; holding. Estimated tip error now: {error:.2f} mm (may settle). This is not independently measured accuracy.")
            self.state()
        return requested

    def control_tick(self, dt, now):
        desired = self.desired(dt, now)
        endpoint = (full_pose(self.motion["plan"].waypoints[-1], self.motion["reference"])
                    if self.motion is not None else desired)
        return self.controller.tick(desired, dt, self.args.joint_speed, 2.0, now, endpoint_target=endpoint)

    def handle(self, request, payload):
        if request == "connect":
            self.connect()
        elif request == "enable":
            if not self.controller:
                raise ValueError("Connect first")
            actual = self.controller.enable()
            self.pending = self.motion = None
            self.faulted = False
            self.emit("invalidate")
            self.emit("target", self.kinematics.xyz(body_vector(actual)))
            self.emit("status", "Holding. Click an XYZ jog to move directly, or enter XYZ and click Move to XYZ.")
            self.state()
        elif request == "current":
            if not self.controller:
                raise ValueError("Connect first")
            reference = self.controller.commanded if self.controller.enabled else self.controller.pose()
            self.emit("target", self.kinematics.xyz(body_vector(reference)))
            self.pending = None
            self.emit("invalidate")
        elif request == "preview":
            self.preview(*payload)
        elif request == "jog":
            if not self.controller or not self.controller.enabled:
                raise ValueError("Enable holding before jogging")
            axis, delta, direction, revision = payload
            reference = self.controller.commanded if self.controller.enabled else self.controller.pose()
            target = self.kinematics.xyz(body_vector(reference))
            target[axis] += delta
            self.preview(target, direction, revision)
            self.start_motion(self.pending[0])
        elif request == "goto":
            if not self.controller or not self.controller.enabled:
                raise ValueError("Enable holding before moving")
            self.preview(*payload)
            self.start_motion(self.pending[0])
        elif request == "move":
            self.start_motion(payload)
        elif request == "hold":
            self.motion = self.pending = None
            self.emit("invalidate")
            self.state()
            self.emit("status", "Movement cancelled; holding the last commanded joint targets.")

    def run(self):
        last, last_health, last_display = time.monotonic(), 0.0, 0.0
        try:
            while not self.close_event.is_set():
                try:
                    if self.stop_event.is_set():
                        self.stop_event.clear()
                        self.stop()
                        self.emit("status", "Torque off verified. Support the arm before enabling again.")
                    try:
                        request, payload = self.requests.get_nowait()
                    except queue.Empty:
                        request = None
                    if request:
                        # Rejected previews are normal UI feedback, not motor faults.
                        try:
                            self.handle(request, payload)
                        except ValueError as exc:
                            if request not in ("preview", "jog", "goto", "move", "current"):
                                raise
                            self.pending = None
                            self.emit("invalidate")
                            self.emit("status", f"Request rejected: {exc}")
                            self.state()
                    now = time.monotonic()
                    dt, last = min(0.1, now - last), now
                    if self.controller:
                        if self.controller.enabled and now - self.heartbeat > 0.75:
                            raise RuntimeError("UI heartbeat lost")
                        if self.controller.enabled and not self.faulted:
                            if self.stop_event.is_set() or self.close_event.is_set():
                                continue
                            actual = self.control_tick(dt, now)
                            if now - last_health >= 0.5:
                                self.controller.health()
                                last_health = now
                        else:
                            try:
                                actual = self.controller.pose()
                            except Exception:
                                if not self.faulted:
                                    raise
                                self.emit("read_error")
                                self.close_event.wait(0.2)
                                continue
                        if now - last_display >= 0.1:
                            xyz = self.kinematics.xyz(body_vector(actual))
                            command = self.controller.commanded if self.controller.enabled else actual
                            commanded_xyz = self.kinematics.xyz(body_vector(command))
                            self.emit("pose", (actual, xyz, commanded_xyz, self.controller.endpoint_offsets.copy()))
                            if self.writer:
                                for key, value in actual.items():
                                    self.writer.writerow([now, "moving" if self.motion else "holding" if self.controller.enabled else "torque_off",
                                                          key, command[key], value, *xyz])
                                self.log.flush()
                            last_display = now
                except Exception as exc:
                    self.faulted = True
                    cleanup = ""
                    try:
                        self.stop()
                    except Exception as stop_error:
                        cleanup = f" Torque-off could not be confirmed: {stop_error}"
                    self.state()
                    self.emit("status", f"STOPPED: {exc}.{cleanup}")
                    print(f"STOPPED: {exc}.{cleanup}", flush=True)
                    # Do not flood the console on a disconnected cable.
                    if self.controller is None:
                        try:
                            if self.robot and self.robot.bus.is_connected:
                                try:
                                    self.robot.disconnect()
                                finally:
                                    if self.robot.bus.is_connected:
                                        self.robot.bus.port_handler.closePort()
                        except Exception as disconnect_error:
                            print(f"Disconnect failed: {disconnect_error}", flush=True)
                        self.robot = None
                self.close_event.wait(0.05 if self.controller and self.controller.enabled else 0.2)
        finally:
            try:
                self.stop()
            except Exception as exc:
                print(f"Shutdown failed: {exc}. Support the arm and disconnect motor power.", flush=True)
            try:
                if self.robot and self.robot.bus.is_connected:
                    try:
                        self.robot.disconnect()
                    finally:
                        if self.robot.bus.is_connected:
                            self.robot.bus.port_handler.closePort()
            finally:
                if self.log:
                    self.log.close()
                self.emit("closed")


class CartesianApp:
    def __init__(self, root, args):
        self.root = root
        self.worker = CartesianWorker(args)
        self.connected = self.enabled = self.moving = self.locked = self.closing = False
        self.preview_id = None
        self.revision = 0
        self.setting_fields = False
        self.xyz = [tk.StringVar(value="0.000") for _ in range(3)]
        self.hold_direction = tk.BooleanVar(value=False)
        self.step = tk.StringVar(value="1")
        self.status = tk.StringVar(value="Connect to read positions; no motor power is enabled by Connect.")
        self.actual_text = tk.StringVar(value="Estimated XYZ: not connected")
        self.command_text = tk.StringVar(value="Held target XYZ: not connected")
        self.joint_text = tk.StringVar(value="")
        self.preview_text = tk.StringVar(value="No preview")
        root.title("SO101 XYZ test" + (" - DEMO: NO HARDWARE" if args.demo else ""))
        root.minsize(850, 660)
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="XYZ tip jogging", font=("Segoe UI", 17, "bold")).pack(anchor="w")
        ttk.Label(frame, text="Millimetres in the URDF base frame. Tip = gripper reference + local tool offset.").pack(anchor="w")
        ttk.Label(frame, text=f"Tool offset: {args.tool_offset_mm} mm | P: shoulder {args.shoulder_p}, elbow {args.elbow_p} | Tip speed <= {args.tip_speed:g} mm/s").pack(anchor="w")
        ttk.Label(frame, text="Model estimates are not measured tip accuracy. Check axes with 1 mm jogs. No collision avoidance.").pack(anchor="w", pady=5)
        ttk.Label(frame, textvariable=self.actual_text).pack(anchor="w")
        ttk.Label(frame, textvariable=self.command_text).pack(anchor="w")
        ttk.Label(frame, textvariable=self.joint_text, wraplength=810).pack(anchor="w", pady=6)
        inputs = ttk.Frame(frame)
        inputs.pack(fill="x", pady=6)
        for index, axis in enumerate("XYZ"):
            ttk.Label(inputs, text=f"{axis} (mm)").grid(row=0, column=index * 2, padx=4)
            ttk.Entry(inputs, textvariable=self.xyz[index], width=13).grid(row=0, column=index * 2 + 1, padx=4)
            self.xyz[index].trace_add("write", self.invalidate)
        ttk.Checkbutton(frame, text="Preserve tip direction (may reject some XYZ targets)", variable=self.hold_direction,
                        command=self.invalidate).pack(anchor="w")
        jogs = ttk.Frame(frame)
        jogs.pack(fill="x", pady=8)
        ttk.Label(jogs, text="Click-to-move distance (mm):").pack(side="left")
        ttk.Combobox(jogs, textvariable=self.step, values=("0.5", "1", "2", "5"), width=5, state="readonly").pack(side="left", padx=6)
        self.jog_buttons = []
        for index, axis in enumerate("XYZ"):
            for sign in (-1, 1):
                button = ttk.Button(jogs, text=f"{'−' if sign < 0 else '+'}{axis}", width=5,
                                    command=lambda i=index, s=sign: self.jog(i, s))
                button.pack(side="left", padx=3)
                self.jog_buttons.append(button)
        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=6)
        self.connect_button = ttk.Button(buttons, text="Connect", command=lambda: self.send("connect"))
        self.enable_button = ttk.Button(buttons, text="Enable / hold", command=self.enable)
        self.current_button = ttk.Button(buttons, text="Use held/current XYZ", command=lambda: self.send("current"))
        self.preview_button = ttk.Button(buttons, text="Preview only", command=self.preview)
        self.move_button = ttk.Button(buttons, text="Move to XYZ", command=self.move)
        for button in (self.connect_button, self.enable_button, self.current_button, self.preview_button, self.move_button):
            button.pack(side="left", padx=(0, 6))
        self.hold_button = ttk.Button(frame, text="Cancel motion / keep holding", command=lambda: self.send("hold"))
        self.hold_button.pack(anchor="w", pady=4)
        tk.Button(frame, text="STOP - TORQUE OFF (Esc) — support the arm", bg="#b42318", fg="white", command=self.stop).pack(fill="x", pady=5)
        ttk.Label(frame, textvariable=self.preview_text, wraplength=810).pack(anchor="w", pady=5)
        self.canvas = tk.Canvas(frame, height=160, background="#f6f7f9", highlightthickness=0)
        self.canvas.pack(fill="x")
        ttk.Label(frame, textvariable=self.status, wraplength=810).pack(anchor="w", pady=8)
        root.bind("<Escape>", lambda event: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.controls()
        self.worker.start()
        root.after(50, self.poll)

    def controls(self):
        available = self.connected and not self.moving and not self.closing
        self.connect_button.configure(state="disabled" if self.connected or self.closing else "normal")
        self.enable_button.configure(state="normal" if available and not self.enabled and not self.locked else "disabled")
        for button in (self.current_button, self.preview_button):
            button.configure(state="normal" if available and not self.locked else "disabled")
        for button in (self.move_button, *self.jog_buttons):
            button.configure(state="normal" if available and self.enabled and not self.locked else "disabled")
        self.hold_button.configure(state="normal" if self.enabled and not self.closing else "disabled")

    def invalidate(self, *_):
        if self.setting_fields:
            return
        self.preview_id = None
        self.revision += 1
        self.preview_text.set("Target edited. Move to XYZ validates and moves; Preview only sends no motion.")
        if hasattr(self, "move_button"):
            self.controls()

    def send(self, request, payload=None):
        self.worker.requests.put((request, payload))

    def enable(self):
        if messagebox.askokcancel("Enable holding", "Support the arm and clear its path. All joints will hold their current positions. Enable torque?", parent=self.root):
            self.send("enable")

    def preview(self):
        try:
            target = [float(value.get()) for value in self.xyz]
        except ValueError:
            self.status.set("Enter finite X, Y, and Z values in millimetres.")
            return
        self.invalidate()
        self.send("preview", (target, self.hold_direction.get(), self.revision))

    def jog(self, axis, sign):
        self.invalidate()
        self.moving = True
        self.status.set("Validating jog, then moving…")
        self.controls()
        self.send("jog", (axis, sign * float(self.step.get()), self.hold_direction.get(), self.revision))

    def move(self):
        try:
            target = [float(value.get()) for value in self.xyz]
        except ValueError:
            self.status.set("Enter finite X, Y, and Z values in millimetres.")
            return
        self.invalidate()
        self.moving = True
        self.status.set("Validating XYZ path, then moving…")
        self.controls()
        self.send("goto", (target, self.hold_direction.get(), self.revision))

    def stop(self):
        self.worker.stop_event.set()
        self.preview_id = None
        self.status.set("Torque-off requested. Support the arm.")

    def close(self):
        self.closing = True
        self.worker.close_event.set()
        self.worker.stop_event.set()
        self.status.set("Stopping, restoring gains, and closing serial connection…")
        self.controls()

    def set_target(self, values):
        self.setting_fields = True
        for variable, value in zip(self.xyz, values):
            variable.set(f"{value:.3f}")
        self.setting_fields = False

    def draw_preview(self, points):
        self.canvas.delete("all")
        points = np.asarray(points)
        width = max(800, self.canvas.winfo_width())
        for panel, axes, title in ((0, (0, 1), "XY preview"), (1, (0, 2), "XZ preview")):
            left = panel * width / 2
            p = points[:, axes]
            span = np.maximum(np.ptp(p, axis=0), 4)
            centre = np.mean([p.min(axis=0), p.max(axis=0)], axis=0)
            scale = min((width / 2 - 70) / span[0], 110 / span[1])
            xy = (p - centre) * [scale, -scale] + [left + width / 4, 90]
            self.canvas.create_text(left + 12, 12, text=title + " (model only)", anchor="w")
            if len(xy) > 1:
                self.canvas.create_line(*xy.flatten(), fill="#2166ac", width=2)
            for point, colour in ((xy[0], "#15803d"), (xy[-1], "#b42318")):
                self.canvas.create_oval(point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4, fill=colour, outline="")

    def poll(self):
        self.worker.heartbeat = time.monotonic()
        while not self.worker.updates.empty():
            kind, payload = self.worker.updates.get_nowait()
            if kind == "state":
                self.connected, self.enabled, self.locked, self.moving = (payload[name] for name in ("connected", "enabled", "locked", "moving"))
            elif kind == "status":
                self.status.set(payload)
            elif kind == "invalidate":
                self.preview_id = None
            elif kind == "target":
                self.set_target(payload)
                self.preview_id = None
            elif kind == "pose":
                pose, xyz, commanded, offsets = payload
                self.actual_text.set("Estimated actual XYZ (mm): " + ", ".join(f"{axis}={value:.2f}" for axis, value in zip("XYZ", xyz)))
                self.command_text.set("Held/commanded XYZ (mm): " + ", ".join(f"{axis}={value:.2f}" for axis, value in zip("XYZ", commanded)))
                self.joint_text.set(" | ".join(f"{key.removesuffix('.pos')}: {value:.2f}" for key, value in pose.items())
                                   + (" — Holding with residual joint error; Cartesian accuracy is not confirmed." if offsets else ""))
            elif kind == "preview":
                if payload["revision"] != self.revision:
                    continue
                self.set_target(payload["target"])
                self.preview_id = payload["id"]
                change = np.max(np.abs(payload["end"] - payload["start"]))
                self.preview_text.set(f"Validated path: {len(payload['points']) - 1} IK intervals, smoothly blended; largest total joint change {change:.2f}°. Gripper opening stays fixed. Green=start, red=target.")
                self.draw_preview(payload["points"])
            elif kind == "read_error":
                self.actual_text.set("Estimated XYZ unavailable: serial read failed")
            elif kind == "closed":
                self.root.destroy()
                return
            self.controls()
        self.root.after(50, self.poll)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--robot-id", default="so101_follower_TCH")
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--tool-offset-mm", type=float, nargs=3, default=(0, 0, 0), metavar=("X", "Y", "Z"))
    parser.add_argument("--shoulder-p", type=int, choices=(16, 20, 32, 36, 40, 48), default=48)
    parser.add_argument("--elbow-p", type=int, choices=(16, 20, 32, 36, 40, 48), default=36)
    parser.add_argument("--tip-speed", type=float, default=1.0, help="Nominal maximum model-tip speed in mm/s (0.1–5; default 1).")
    parser.add_argument("--joint-speed", type=float, default=2 / 3, help="Joint speed limit in degrees/s (0.1–2; default 0.667).")
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.tip_speed) or not 0.1 <= args.tip_speed <= 5:
        parser.error("Tip speed must be 0.1–5 mm/s")
    if not math.isfinite(args.joint_speed) or not 0.1 <= args.joint_speed <= 2:
        parser.error("Joint speed must be 0.1–2 degrees/s")
    root = tk.Tk()
    app = CartesianApp(root, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()
    finally:
        app.worker.close_event.set()
        app.worker.stop_event.set()
        app.worker.join(timeout=5)
        if app.worker.is_alive():
            print("Serial cleanup is still running. Support the arm; disconnect motor power if torque remains enabled.", flush=True)


if __name__ == "__main__":
    main()
