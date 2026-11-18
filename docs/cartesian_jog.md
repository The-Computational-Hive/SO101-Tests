# Separate XYZ jogging test

The existing joint-slider test is unchanged. `scripts/cartesian_jog.py` adds a
separate app using the official SO101 URDF and NumPy. No additional pip packages,
ROS, Placo, or SciPy are needed in the current environment.

## Preview without hardware

```powershell
.\.venv\Scripts\python.exe scripts\cartesian_jog.py --demo
```

Click Connect, Enable / hold, then +X with the 1 mm jog distance. Each jog click
validates the short path and starts motion directly (simulated in demo mode).
For absolute coordinates, enter XYZ and click Move to XYZ. Editing inputs alone
does not move the arm. Preview only remains available for inspection without
motion, including while torque is off; moving always validates a fresh path.

## Use the real arm

Close the joint-slider app so COM3 is available. Keep the verified power-supply
setting, secure the base, and clear the arm's path. This test moves the empty
gripper; start without a pen or contact with a surface.

```powershell
.\.venv\Scripts\python.exe scripts\cartesian_jog.py
```

1. Connect: reads/configures the robot but leaves torque off.
2. Check the estimated XYZ against the physical gripper and the model axes.
   These are model-derived readings, not independent tip measurements.
3. Support the arm and click Enable / hold. Wait for a steady hold.
4. Select a 1 mm jog distance. +X, -X, +Y, -Y, +Z, and -Z each validate and
   execute a move relative to the held target on one click. No second click or
   per-move confirmation is needed. Preview only is optional.
5. Check the physical direction and magnitude of each axis before larger moves.
6. Cancel motion / keep holding freezes the last commanded goals. Esc or STOP
   requests torque off. Support the arm before torque off; disconnect motor
   power if OFF cannot be confirmed.

Absolute X/Y/Z inputs are millimetres in the URDF `base_link` frame, not camera,
paper, or tool-local coordinates. Every planned move is limited to 10 mm from the
held target (or the current measured pose for previews while torque is off). There is no
automatic home-to-zero move. Use held/current XYZ populates the inputs without
moving the arm. While enabled, it uses the held command rather than erasing a
gravity-related holding error by retargeting to a sagged position.

## Orientation and tool reference

The default target is the URDF `gripper_frame_link` origin. The gripper opening
stays fixed. XYZ-only solving can change tool orientation. The optional Preserve
tip direction mode tries to retain the initial local Z direction and rejects
motions it cannot achieve; it does not constrain rotation about that direction.
Five arm joints cannot satisfy every arbitrary six-dimensional tool pose.

For a rigid tool, provide its tip displacement in the local gripper reference:

```powershell
.\.venv\Scripts\python.exe scripts\cartesian_jog.py --tool-offset-mm 0 0 50
```

This is an example offset, not a measured recommendation for a particular pen.
Model zero conventions, signs, dimensions, and tool mounting must agree with the
physical arm. If the first tiny jog moves incorrectly, stop and investigate
model/calibration agreement rather than increasing gains or error allowances.

## Existing motion settings retained

- Port COM3 and robot ID `so101_follower_TCH` by default.
- Shoulder P=48, elbow P=36, temporary and restored on shutdown.
- Shoulder/elbow target lead 3 degrees, other leads 2.5, and matching LeRobot
  backup clipping bounds. Two-degree body endpoint allowance remains visibly
  reported as residual joint error, not Cartesian success.
- Endpoint classification uses the whole trajectory's final joint destination,
  not its intermediate setpoints. It allows one encoder count (about 0.088 deg)
  when checking whether commanded/setpoint angles have reached that destination;
  actual residual error must still remain within the existing two-degree bound.
  Failure output includes `final_target` to distinguish mid-path and endpoint cases.
- Joint-speed limit 0.667 degrees/s and nominal model-tip speed 1 mm/s.
- Motor faults, unexpected torque states, goal readback mismatch, temperature
  cutoff, stall checks, and the UI-heartbeat watchdog remain active.
- All motor traffic runs on one worker. Shutdown attempts every motor and reads
  each torque state independently. Unconfirmed shutdown locks out enabling.

Optional `--joint-speed` accepts 0.1–2 degrees/s and `--tip-speed` accepts
0.1–5 mm/s. Start with the defaults. The joint limit can make execution slower
than the nominal tip-speed setting.

## What is validated

Before execution, the entire short Cartesian path is solved into nearby joint
waypoints, with calibration/model limit intersection, at most 0.5 mm between
Cartesian samples, at most 2 degrees between IK waypoints, and at most 10 degrees
of total movement per joint. Unreachable targets are rejected. The planner checks
model position residual (0.15 mm) and, when requested, tool-axis residual (0.5 deg).
These numerical tolerances do not imply comparable physical accuracy.

Joint targets follow one continuous cubic Hermite path with non-overshooting
joint interpolation and continuous velocity at internal waypoints. The path
starts and ends at rest rather than deliberately stopping at each 0.5 mm sample.
Timing is scaled using exact joint-speed extrema and sampled model-tip speed.
The smoothed model path is checked for deviations over 0.3 mm from the line and,
when requested, tool-direction deviations over 1 degree. No SciPy dependency
is required; internal tangents use the weighted harmonic-mean construction
described in the [PCHIP documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.PchipInterpolator.html), with zero endpoint velocities.

The existing slew and lead limits remain active. Path progression pauses when
commands cannot keep up. New movement clicks are disabled while moving, rather
than accumulating a hidden motion queue. At the endpoint, commands finish and
the arm keeps holding; there is no mandatory two-second wait before the next jog.
A freshly planned move is rejected if the measured arm drifts more than 1 degree
before execution. There is no automatic fault recovery motion or return.

The UI shows estimated actual and held-target XYZ separately, plots XY/XZ path
projections, and reports estimated final tip error. Actual joint error, backlash,
tool compliance, and model inaccuracies remain. This test is not a force
controller and has no collision avoidance. Even a small path may intersect the
table or another object; keep the first tests in free space.

Telemetry is saved under the ignored `logs/xyz_*.csv` paths. Offline tests cover
forward-kinematics axis behavior, IK solutions and limits, path continuity,
unreachable/nonfinite targets, cancellation, fixed gripper opening, stale
previews, slew limits, and heartbeat shutdown. No physical arm was moved during
implementation or verification.
