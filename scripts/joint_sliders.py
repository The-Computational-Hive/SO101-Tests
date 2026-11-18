"""SO101 joint sliders. Run with --demo to preview without connecting hardware."""

import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    from .test_api import SafeStartFollower, read_pose
except ImportError:
    from test_api import SafeStartFollower, read_pose
from lerobot.robots.so_follower import SO101FollowerConfig


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def calibrated_limits(calibration, margin=2.0):
    """Match LeRobot's degree normalization; gripper remains on a 0-100 scale."""
    limits = {}
    for joint in JOINTS:
        entry = calibration[joint]
        if entry.range_max <= entry.range_min:
            raise ValueError(f"Invalid calibration for {joint}")
        half = (entry.range_max - entry.range_min) * 180 / 4095
        low, high = (0.0, 100.0) if joint == "gripper" else (-half, half)
        joint_margin = 0.0 if joint == "gripper" else margin
        if high - low <= 2 * joint_margin:
            raise ValueError(f"Calibrated range too narrow for {joint}")
        limits[f"{joint}.pos"] = (low + joint_margin, high - joint_margin)
    return limits


def bounded_targets(targets, limits):
    result = {}
    for key, (low, high) in limits.items():
        value = float(targets[key])
        if not math.isfinite(value):
            raise ValueError(f"Invalid target for {key}")
        result[key] = min(high, max(low, value))
    return result


def ramp_targets(commanded, desired, dt, body_speed, gripper_speed):
    # Cap elapsed time so a delayed iteration never creates a large jump.
    dt = max(0.0, min(dt, 0.1))
    result = {}
    for key, value in commanded.items():
        step = (gripper_speed if key == "gripper.pos" else body_speed) * dt
        result[key] = value + max(-step, min(step, desired[key] - value))
    return result


def disable_all_torque(bus):
    """Attempt every motor even when an earlier write fails; verify individually."""
    write_errors = {}
    for joint in JOINTS:
        try:
            bus.disable_torque(joint, num_retry=1)
        except Exception as exc:
            write_errors[joint] = str(exc)
    failures = []
    for joint in JOINTS:
        try:
            value = bus.read("Torque_Enable", joint, normalize=False, num_retry=1)
            if value != 0:
                raise RuntimeError(f"Torque_Enable={value}")
        except Exception as exc:
            detail = f"{joint}: {exc}"
            if joint in write_errors:
                detail += f"; write failed: {write_errors[joint]}"
            failures.append(detail)
    if failures:
        raise RuntimeError("Torque-off failed; could not confirm OFF on " + "; ".join(failures))
    if write_errors:
        print("Torque-off verified on all motors despite acknowledgement errors on: "
              + ", ".join(write_errors), flush=True)


class ArmController:
    """All serial calls run on one worker thread, never in Tk callbacks."""

    def __init__(self, robot, limits, gains, max_lead=None, settle_tolerance=1.5):
        self.robot, self.limits, self.gains = robot, limits, gains
        self.max_lead = {key: 2.5 for key in limits}
        self.max_lead.update(max_lead or {})
        if set(self.max_lead) != set(limits) or any(value not in (2.5, 3.0) for value in self.max_lead.values()):
            raise ValueError("Command lead must be 2.5 or 3.0 for known joints")
        if self.max_lead.get("gripper.pos", 2.5) != 2.5:
            raise ValueError("Gripper command lead must remain 2.5 units")
        if settle_tolerance not in (1.5, 2.0):
            raise ValueError("Body endpoint tolerance must be 1.5 or 2.0 degrees")
        self.settle_tolerance = settle_tolerance
        self.endpoint_offsets = {}
        self.enabled = False
        self.original_gains = {}
        self.commanded = {}
        self.lag_since = {}
        self.shutdown_failed = False

    def tracking_failure(self, key, actual, desired, reason, final_target=None):
        joint = key.removesuffix(".pos")
        unit = "units" if joint == "gripper" else "degrees"
        details = (f"{joint}: {reason}; desired={desired[key]:.3f}, "
                   f"commanded={self.commanded[key]:.3f}, actual={actual[key]:.3f}, "
                   f"error={actual[key] - self.commanded[key]:+.3f} {unit}, "
                   f"lead_limit={self.max_lead[key]:.1f}")
        if final_target is not None:
            details += f", final_target={final_target:.3f}"
        readings = []
        for register in ("Goal_Position", "Torque_Enable", "P_Coefficient", "I_Coefficient", "D_Coefficient",
                         "Max_Torque_Limit", "Torque_Limit", "Protection_Current",
                         "Protective_Torque", "Overload_Torque", "Protection_Time",
                         "Present_Voltage", "Present_Current", "Present_Load", "Status"):
            try:
                value = self.robot.bus.read(register, joint, normalize=(register == "Goal_Position"))
                readings.append(f"{register}={value}")
            except Exception as exc:
                readings.append(f"{register}=unavailable ({exc})")
        print(details + "\n  " + ", ".join(readings), flush=True)
        raise RuntimeError(details)

    def pose(self):
        return read_pose(self.robot)

    def enable(self):
        if self.shutdown_failed:
            raise RuntimeError("Previous torque-off failed. Disconnect motor power, inspect connections, and restart the UI.")
        if self.enabled:
            return self.commanded.copy()
        if self.original_gains:
            raise RuntimeError("Gain restoration is incomplete. Stop again before enabling.")
        actual = self.pose()
        for key, (low, high) in self.limits.items():
            if not low <= actual[key] <= high:
                raise RuntimeError(f"{key} is outside the slider limits. Reposition with torque off.")
        try:
            disable_all_torque(self.robot.bus)
            for joint, gain in self.gains.items():
                self.original_gains[joint] = self.robot.bus.read("P_Coefficient", joint, normalize=False)
                self.robot.bus.write("P_Coefficient", joint, gain, normalize=False)
                if self.robot.bus.read("P_Coefficient", joint, normalize=False) != gain:
                    raise RuntimeError(f"P gain readback failed on {joint}")
            # Read again after setup. Goal writes may themselves enable torque.
            actual = self.pose()
            if bounded_targets(actual, self.limits) != actual:
                raise RuntimeError("Arm moved outside slider limits during startup.")
            self.robot.send_action(actual)
            stored = self.robot.bus.sync_read("Goal_Position")
            for joint in JOINTS:
                if abs(stored[joint] - actual[f"{joint}.pos"]) > 0.2:
                    raise RuntimeError(f"Hold target readback failed on {joint}")
            self.robot.bus.enable_torque()
            self.commanded = actual.copy()
            self.lag_since.clear()
            self.endpoint_offsets.clear()
            self.enabled = True
            return actual
        except Exception:
            self.stop()
            raise

    def stop(self):
        self.enabled = False  # Prevent further actions even if serial cleanup fails.
        self.endpoint_offsets.clear()
        errors = []
        try:
            disable_all_torque(self.robot.bus)
        except Exception as exc:
            self.shutdown_failed = True
            raise RuntimeError(f"{exc}. Support the arm and disconnect motor power; restart the UI after checking connections.") from exc
        for joint, gain in list(self.original_gains.items()):
            try:
                self.robot.bus.write("P_Coefficient", joint, gain, normalize=False)
                if self.robot.bus.read("P_Coefficient", joint, normalize=False) != gain:
                    raise RuntimeError("readback mismatch")
                del self.original_gains[joint]
            except Exception as exc:
                errors.append(f"Could not restore {joint} P: {exc}")
        self.lag_since.clear()
        if errors:
            raise RuntimeError("; ".join(errors))

    def tick(self, desired, dt, speed, gripper_speed, now, *, endpoint_target=None):
        actual = self.pose()
        if not self.enabled:
            return actual
        desired = bounded_targets(desired, self.limits)
        endpoint = desired if endpoint_target is None else bounded_targets(endpoint_target, self.limits)
        candidate = ramp_targets(self.commanded, desired, dt, speed, gripper_speed)
        self.endpoint_offsets.clear()
        for key, value in self.commanded.items():
            error = abs(actual[key] - value)
            if error > 5.0:
                self.tracking_failure(key, actual, desired, "tracking error exceeded 5", endpoint[key])
            # Cartesian callers stream intermediate setpoints. Compare against
            # the actual final destination, with one encoder count of tolerance,
            # rather than requiring floating-point equality with each setpoint.
            endpoint_resolution = 360 / 4095 + 1e-6
            at_endpoint = (abs(endpoint[key] - value) <= endpoint_resolution
                           and abs(endpoint[key] - desired[key]) <= endpoint_resolution)
            final_error = abs(actual[key] - endpoint[key])
            small_final_offset = (key != "gripper.pos" and at_endpoint
                                  and 1.5 < error <= self.settle_tolerance
                                  and final_error <= self.settle_tolerance)
            if small_final_offset:
                # This is an explicit endpoint-accuracy allowance, not success.
                # Keep the target and effort unchanged, display the residual,
                # and continue all motor/temperature/communication checks.
                self.endpoint_offsets[key] = actual[key] - endpoint[key]
                self.lag_since.pop(key, None)
            elif error > 1.5:
                direction = 1 if value > actual[key] else -1
                anchor = self.lag_since.get(key)
                # Measure physical progress toward the command, not just absolute
                # error: gravity can produce a steady offset during useful motion.
                if (anchor is None or anchor[2] != direction
                        or (actual[key] - anchor[1]) * direction >= 0.25):
                    self.lag_since[key] = (now, actual[key], direction)
                elif (candidate[key] - value) * direction < 0:
                    # A cancellation/reversal is reducing the commanded error.
                    self.lag_since.pop(key, None)
                elif now - anchor[0] > 3.0:
                    self.tracking_failure(key, actual, desired, "not tracking: less than 0.25 units of progress in 3 seconds", endpoint[key])
            else:
                self.lag_since.pop(key, None)
            # Keep the command inside its selected lead allowance and below
            # LeRobot's per-joint cap (lead + 0.5). Never jump
            # the command toward a sagging joint; preserve the speed limit.
            next_error = abs(candidate[key] - actual[key])
            if next_error > self.max_lead[key] and next_error > error:
                candidate[key] = value
        sent = self.robot.send_action(candidate)
        if any(abs(sent[key] - candidate[key]) > 0.2 for key in candidate):
            raise RuntimeError("LeRobot clipped a target; stopping.")
        self.commanded = candidate
        return actual

    def health(self):
        for register in ("Torque_Enable", "Status", "Present_Temperature", "Goal_Position"):
            values = self.robot.bus.sync_read(register, normalize=(register == "Goal_Position"))
            for joint, value in values.items():
                if register == "Torque_Enable" and value != 1:
                    raise RuntimeError(f"Torque unexpectedly disabled on {joint}")
                if register == "Status" and value != 0:
                    description = " (input voltage error)" if int(value) & 1 else ""
                    readings = []
                    for field in ("Present_Voltage", "Min_Voltage_Limit", "Max_Voltage_Limit"):
                        try:
                            reading = self.robot.bus.read(field, joint, normalize=False)
                            readings.append(f"{field}={reading}")
                        except Exception as exc:
                            readings.append(f"{field}=unavailable ({exc})")
                    print(f"Motor fault on {joint}: {value}{description}; " + ", ".join(readings), flush=True)
                    raise RuntimeError(f"Motor fault on {joint}: {value}{description}")
                if register == "Present_Temperature" and value >= 50:
                    raise RuntimeError(f"Temperature reached 50 C on {joint}; stopping.")
                if register == "Goal_Position" and abs(value - self.commanded[f"{joint}.pos"]) > 0.2:
                    raise RuntimeError(f"Stored goal mismatch on {joint}")


class SliderApp:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.requests, self.updates = queue.Queue(), queue.Queue()
        self.stop_event, self.close_event = threading.Event(), threading.Event()
        self.reset_ready = threading.Event()
        self.lock = threading.Lock()
        self.shared = {"targets": {}, "speed": 2 / 3, "gripper_speed": 2.0, "heartbeat": time.monotonic()}
        self.connected = self.enabled = self.closing = self.locked_out = False
        self.status = tk.StringVar(value="Connect, then support the arm and enable holding.")
        self.offset_warning = tk.StringVar(value="")
        self.speed = tk.DoubleVar(value=2 / 3)
        self.gripper_speed = tk.DoubleVar(value=2.0)
        self.values, self.readouts, self.sliders, self.limit_labels = {}, {}, {}, {}
        root.title("SO101 joint sliders" + (" — DEMO" if args.demo else ""))
        root.minsize(840, 510)
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="SO101 joint control", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(frame, text=f"{args.port}  |  {args.robot_id}  |  Shoulder P={args.shoulder_p}, elbow P={args.elbow_p}").pack(anchor="w")
        ttk.Label(frame, text=f"Command lead: shoulder ≤{args.shoulder_lead:g}°, elbow ≤{args.elbow_lead:g}°; others retain 2.5.").pack(anchor="w")
        ttk.Label(frame, text=f"Body endpoint allowance: {args.settle_tolerance:g}°; larger offsets still trigger tracking checks.").pack(anchor="w")
        ttk.Label(frame, text="Secure the base. Limits do not prevent collisions. Support the arm before torque off.").pack(anchor="w", pady=(4, 12))
        rows = ttk.Frame(frame)
        rows.pack(fill="x")
        rows.columnconfigure(1, weight=1)
        for index, joint in enumerate(JOINTS):
            key = f"{joint}.pos"
            self.values[key] = tk.DoubleVar(value=0)
            self.readouts[key] = tk.StringVar(value="Not connected")
            self.limit_labels[key] = ttk.Label(rows, text=joint, width=23)
            self.limit_labels[key].grid(row=index, column=0, sticky="w", pady=8)
            slider = ttk.Scale(rows, variable=self.values[key], from_=-1, to=1, state="disabled")
            slider.grid(row=index, column=1, sticky="ew", padx=12)
            self.sliders[key] = slider
            ttk.Label(rows, textvariable=self.readouts[key], width=48).grid(row=index, column=2, sticky="w")
        speeds = ttk.Frame(frame)
        speeds.pack(fill="x", pady=12)
        ttk.Label(speeds, text="Body speed (degrees/s)").grid(row=0, column=0, sticky="w")
        ttk.Scale(speeds, from_=0.1, to=5.0, variable=self.speed, length=260).grid(row=0, column=1, padx=12)
        self.speed_label = ttk.Label(speeds, text="0.67")
        self.speed_label.grid(row=0, column=2)
        ttk.Label(speeds, text="Gripper speed (units/s)").grid(row=1, column=0, sticky="w")
        ttk.Scale(speeds, from_=0.5, to=10.0, variable=self.gripper_speed, length=260).grid(row=1, column=1, padx=12)
        self.gripper_label = ttk.Label(speeds, text="2.00")
        self.gripper_label.grid(row=1, column=2)
        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=8)
        self.connect_button = ttk.Button(buttons, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=(0, 8))
        self.enable_button = ttk.Button(buttons, text="Enable / hold current pose", command=self.enable, state="disabled")
        self.enable_button.pack(side="left", padx=8)
        self.hold_button = ttk.Button(buttons, text="Cancel movement / hold", command=lambda: self.requests.put("hold"), state="disabled")
        self.hold_button.pack(side="left", padx=8)
        tk.Button(buttons, text="STOP — TORQUE OFF (Esc)", bg="#b42318", fg="white", command=self.stop).pack(side="right")
        ttk.Label(frame, textvariable=self.status, wraplength=800).pack(anchor="w", pady=8)
        tk.Label(frame, textvariable=self.offset_warning, fg="#9a5700", anchor="w", justify="left", wraplength=800).pack(anchor="w")
        root.bind("<Escape>", lambda event: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.worker = threading.Thread(target=self.work, daemon=False)
        self.worker.start()
        root.after(50, self.poll)

    def connect(self):
        self.connect_button.configure(state="disabled")
        self.status.set("Connecting with torque off…")
        self.requests.put("connect")

    def enable(self):
        if messagebox.askokcancel("Enable holding", "Support the arm and clear its path.\nAll joints will hold their current positions.\nEnable torque?", parent=self.root):
            self.enable_button.configure(state="disabled")
            self.requests.put("enable")

    def stop(self):
        self.stop_event.set()
        self.enabled = False
        self.set_controls()
        self.status.set("Requesting torque off. Support the arm.")

    def close(self):
        self.closing = True
        self.close_event.set()
        self.stop_event.set()
        self.set_controls()
        self.status.set("Disabling torque, restoring gains, and disconnecting…")

    def set_controls(self):
        for slider in self.sliders.values():
            slider.configure(state="normal" if self.enabled and not self.closing else "disabled")
        self.enable_button.configure(state="normal" if self.connected and not self.enabled and not self.closing and not self.locked_out else "disabled")
        self.hold_button.configure(state="normal" if self.enabled and not self.closing else "disabled")

    def poll(self):
        with self.lock:
            self.shared.update(speed=self.speed.get(), gripper_speed=self.gripper_speed.get(), heartbeat=time.monotonic())
            if self.enabled and self.reset_ready.is_set():
                self.shared["targets"] = {key: value.get() for key, value in self.values.items()}
        self.speed_label.configure(text=f"{self.speed.get():.2f}")
        self.gripper_label.configure(text=f"{self.gripper_speed.get():.2f}")
        while not self.updates.empty():
            kind, data = self.updates.get_nowait()
            if kind == "limits":
                self.connected = True
                for key, (low, high) in data.items():
                    self.sliders[key].configure(from_=low, to=high)
                    unit = "units" if key == "gripper.pos" else "°"
                    self.limit_labels[key].configure(text=f"{key.removesuffix('.pos')}\n{low:.1f} to {high:.1f} {unit}")
            elif kind == "state":
                self.enabled = data
                if not data:
                    self.offset_warning.set("")
            elif kind == "lockout":
                self.locked_out = True
                self.enabled = False
            elif kind == "reset":
                for key, value in data.items():
                    self.values[key].set(value)
                # Publish before the worker resumes so stale sliders cannot move the arm.
                with self.lock:
                    self.shared["targets"] = data.copy()
                self.reset_ready.set()
            elif kind == "pose":
                actual, commanded, limits = data
                for key, value in actual.items():
                    unit = "u" if key == "gripper.pos" else "°"
                    self.readouts[key].set(f"Actual {value:.1f}{unit} | Target {self.values[key].get():.1f} | Sent {commanded.get(key, value):.1f}")
                    if not self.enabled:
                        self.values[key].set(value)
            elif kind == "offsets":
                if self.enabled and data:
                    details = ", ".join(f"{key.removesuffix('.pos')}: {error:+.2f}°" for key, error in data.items())
                    self.offset_warning.set("Holding with residual position error: " + details + ". Target not reached precisely.")
                else:
                    self.offset_warning.set("")
            elif kind == "status":
                self.status.set(data)
            elif kind == "error":
                self.enabled = False
                self.status.set(data)
                self.connect_button.configure(state="normal" if not self.connected else "disabled")
            elif kind == "read_error":
                for readout in self.readouts.values():
                    readout.set("Position unavailable - serial read failed")
            elif kind == "closed":
                self.root.destroy()
                return
            self.set_controls()
        self.root.after(50, self.poll)

    def work(self):
        controller = None
        robot = None
        faulted = False
        last_tick = last_health = 0.0
        try:
            while not self.close_event.is_set():
                try:
                    if self.stop_event.is_set():
                        # Discard pending enable/move requests when stopping.
                        while not self.requests.empty():
                            self.requests.get_nowait()
                        self.stop_event.clear()
                        self.reset_ready.clear()
                        if controller:
                            controller.stop()
                        faulted = False
                        self.updates.put(("state", False))
                        status = ("Torque off verified. Previous shutdown failure remains latched; inspect connections and restart the UI."
                                  if controller and controller.shutdown_failed
                                  else "Torque off verified. Support the arm before enabling again.")
                        self.updates.put(("status", status))
                    try:
                        request = self.requests.get_nowait()
                    except queue.Empty:
                        request = None
                    if request == "connect" and controller is None:
                        leads = {"shoulder_lift.pos": self.args.shoulder_lead,
                                 "elbow_flex.pos": self.args.elbow_lead}
                        if self.args.demo:
                            robot = DemoRobot()
                            limits = {f"{j}.pos": (0, 100) if j == "gripper" else (-90, 90) for j in JOINTS}
                        else:
                            robot = SafeStartFollower(SO101FollowerConfig(port=self.args.port, id=self.args.robot_id,
                                                                        use_degrees=True,
                                                                        max_relative_target={
                                                                            joint: leads.get(f"{joint}.pos", 2.5) + 0.5
                                                                            for joint in JOINTS
                                                                        }))
                            if not robot.calibration:
                                raise RuntimeError("No saved calibration found for this robot ID.")
                            robot.connect(calibrate=False)
                            if not robot.is_calibrated:
                                raise RuntimeError("Saved calibration does not match the motors.")
                            limits = calibrated_limits(robot.calibration)
                        controller = ArmController(robot, limits, {"shoulder_lift": self.args.shoulder_p, "elbow_flex": self.args.elbow_p},
                                                   max_lead=leads, settle_tolerance=self.args.settle_tolerance)
                        faulted = False
                        self.updates.put(("limits", limits))
                        self.updates.put(("status", "Connected; torque off. Support the arm, then enable."))
                    if request == "enable" and controller and not self.stop_event.is_set():
                        self.reset_ready.clear()
                        actual = controller.enable()
                        faulted = False
                        with self.lock:
                            self.shared["targets"] = actual.copy()
                        self.updates.put(("reset", actual))
                        self.updates.put(("state", True))
                        self.updates.put(("status", "Holding. Sliders move immediately, gradually toward their targets."))
                        last_tick = time.monotonic()
                    if request == "hold" and controller and controller.enabled:
                        self.reset_ready.clear()
                        actual = controller.pose()
                        controller.commanded = actual.copy()
                        controller.lag_since.clear()
                        with self.lock:
                            self.shared["targets"] = actual.copy()
                        self.updates.put(("reset", actual))
                        self.updates.put(("status", "Movement cancelled; holding the measured pose."))
                    now = time.monotonic()
                    if controller and not faulted:
                        with self.lock:
                            shared = {**self.shared, "targets": self.shared["targets"].copy()}
                        if controller.enabled and now - shared["heartbeat"] > 0.75:
                            raise RuntimeError("UI stopped responding; disabling torque.")
                        # Wait for Tk to acknowledge slider resets before allowing motion.
                        if self.stop_event.is_set() or (controller.enabled and not self.reset_ready.is_set()):
                            time.sleep(0.01)
                            continue
                        if controller.enabled and now - last_tick < 0.05:
                            time.sleep(0.01)
                            continue
                        actual = controller.tick(shared["targets"], now - last_tick, shared["speed"], shared["gripper_speed"], now)
                        last_tick = now
                        if controller.enabled and now - last_health >= 0.5:
                            controller.health()
                            last_health = now
                        self.updates.put(("pose", (actual, controller.commanded.copy(), controller.limits)))
                        self.updates.put(("offsets", controller.endpoint_offsets.copy()))
                    elif controller:
                        # Keep measured positions live after a stop, without sending actions.
                        try:
                            actual = controller.pose()
                            self.updates.put(("pose", (actual, {}, controller.limits)))
                        except Exception:
                            self.updates.put(("read_error", None))
                except Exception as exc:
                    faulted = True
                    cleanup_error = ""
                    if controller:
                        try:
                            controller.stop()
                        except Exception as cleanup:
                            cleanup_error = f" Cleanup failed: {cleanup}. Disconnect power if needed."
                        if controller.shutdown_failed:
                            self.updates.put(("lockout", True))
                    elif robot and robot.bus.is_connected:
                        try:
                            robot.disconnect()
                        except Exception as cleanup:
                            cleanup_error = f" Disconnect failed: {cleanup}."
                    self.updates.put(("state", False))
                    self.updates.put(("error", f"Stopped: {exc}.{cleanup_error}"))
                    print(f"Stopped: {exc}.{cleanup_error}", flush=True)
                    if not controller:
                        robot = None
                self.close_event.wait(0.05 if controller and controller.enabled else 0.2)
        finally:
            try:
                if controller:
                    controller.stop()
            except Exception as exc:
                print(f"Shutdown cleanup failed: {exc}; disconnect power if necessary.", flush=True)
            finally:
                try:
                    if robot and robot.bus.is_connected:
                        try:
                            robot.disconnect()
                        finally:
                            # Upstream disconnect may raise before closing the port
                            # when a torque write fails. Still release the serial port.
                            if robot.bus.is_connected:
                                robot.bus.port_handler.closePort()
                except Exception as exc:
                    print(f"Disconnect failed: {exc}", flush=True)
                self.updates.put(("closed", None))


class DemoRobot:
    """Simulated immediate tracking for previewing the UI without serial access."""

    def __init__(self):
        self.bus = self
        self.is_connected = True
        self.action_features = {f"{joint}.pos": float for joint in JOINTS}
        self.positions = {key: 50.0 if key == "gripper.pos" else 0.0 for key in self.action_features}
        self.goals = self.positions.copy()
        self.torque = 0
        self.gains = {joint: 16 for joint in JOINTS}

    def get_observation(self):
        return self.positions.copy()

    @property
    def torque(self):
        return int(any(self.torques.values()))

    @torque.setter
    def torque(self, value):
        self.torques = {joint: value for joint in JOINTS}

    def send_action(self, action):
        self.goals = action.copy()
        self.torque = 1
        self.positions = action.copy()
        return action.copy()

    def disable_torque(self, motors=None, num_retry=0):
        for joint in JOINTS if motors is None else [motors]:
            self.torques[joint] = 0

    def enable_torque(self):
        self.torque = 1

    def read(self, register, joint, normalize=False, num_retry=0):
        if register == "P_Coefficient":
            return self.gains[joint]
        if register == "Goal_Position":
            return self.goals[f"{joint}.pos"]
        return {"Torque_Enable": self.torques[joint], "Present_Voltage": 55,
                "Present_Current": 0, "Present_Load": 0, "Status": 0,
                "Min_Voltage_Limit": 40, "Max_Voltage_Limit": 80,
                "I_Coefficient": 0, "D_Coefficient": 32,
                "Max_Torque_Limit": 500 if joint == "gripper" else 1000,
                "Torque_Limit": 500 if joint == "gripper" else 1000,
                "Protection_Current": 250 if joint == "gripper" else 500,
                "Protective_Torque": 20, "Overload_Torque": 25 if joint == "gripper" else 80,
                "Protection_Time": 200}[register]

    def write(self, register, joint, value, normalize=False):
        self.gains[joint] = value

    def sync_read(self, register, normalize=True):
        if register == "Goal_Position":
            return {joint: self.goals[f"{joint}.pos"] for joint in JOINTS}
        if register == "Torque_Enable":
            return self.torques.copy()
        value = {"Torque_Enable": self.torque, "Status": 0, "Present_Temperature": 26}[register]
        return {joint: value for joint in JOINTS}

    def disconnect(self):
        self.disable_torque()
        self.is_connected = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--robot-id", default="so101_follower_TCH")
    parser.add_argument("--shoulder-p", type=int, choices=(16, 20, 32, 36, 40, 48), default=32)
    parser.add_argument("--elbow-p", type=int, choices=(16, 20, 32, 36, 40, 48), default=36)
    parser.add_argument("--shoulder-lead", type=float, choices=(2.5, 3.0), default=2.5,
                        help="Shoulder command lead in degrees; 3.0 is an opt-in experiment (default: 2.5).")
    parser.add_argument("--elbow-lead", type=float, choices=(2.5, 3.0), default=2.5,
                        help="Elbow command lead in degrees; 3.0 is an opt-in experiment (default: 2.5).")
    parser.add_argument("--settle-tolerance", type=float, choices=(1.5, 2.0), default=1.5,
                        help="Opt in to holding body endpoints with up to 2 degrees of residual error and a visible warning; default: 1.5. Does not apply during trajectories or to gripper.")
    parser.add_argument("--demo", action="store_true", help="Preview sliders without connecting a robot.")
    args = parser.parse_args()
    root = tk.Tk()
    app = SliderApp(root, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()
    finally:
        app.close_event.set()
        app.stop_event.set()
        app.worker.join(timeout=5)
        if app.worker.is_alive():
            print("Waiting for serial cleanup. If the arm remains powered, disconnect its supply.", flush=True)


if __name__ == "__main__":
    main()
