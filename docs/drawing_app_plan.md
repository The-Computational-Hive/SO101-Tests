# Small SO101 drawing app: proposed first version

## Agreed scope

- Real pen on paper lying flat on a table.
- Start with a straight line, square, triangle, and circle.
- Small Windows desktop app using Tkinter, in the existing Python environment.
- Preview and validate a complete path before enabling execution.
- No new power supply is a prerequisite for software development.
- No model training, leader arm, camera, ROS installation, or SVG import in the first version.

This is a plan only. No drawing controller has been implemented, dependencies
installed, or hardware moved as part of planning.

## Constraints from the current experiments

- Shoulder P=48 with a 3-degree lead significantly improved reported movement.
  Elbow P=36 with a 3-degree lead has been tested, with remaining endpoint offsets.
  These are experimental settings, not a universally validated operating profile.
- The manual UI now supports an explicit 2-degree body endpoint allowance with
  a warning. This is unsuitable as the sole drawing-accuracy criterion: small
  angular errors can produce appreciable pen-tip position and height errors.
- Gripper voltage faults and one corrupted reply during torque-off remain
  unresolved. Offline planning can proceed, but physical execution requires
  reliable communication and verified holding/stopping in the selected workspace.
- There is no force sensor or validated current-to-pen-pressure controller.
  Use passive compliance and light contact, not higher current limits to press
  the pen into paper.
- Keep the arm compact. The drawing patch must fit a tested region that avoids
  full extension, joint endpoints, and awkward wrist configurations.

## User interface

One window with:

1. Shape selection, width/height in millimetres, position, and rotation.
2. A top-down preview with the paper patch boundary and visible pen-up travels.
3. Paper/tool setup status and a separate Connect / Enable holding sequence.
4. Validate path, Trace above paper, and Draw controls.
5. Pause/hold and a prominent torque-off stop, clearly distinguished.
6. Current joint readings, path progress, position-error warnings, and fault text.

Changing a shape must not move the robot. Draw requires a valid preview and an
explicit start. A disconnected preview mode is the default development path.
Editing the shape, changing calibration/tool setup, or moving the paper invalidates
the prepared trajectory. A physical dry run changes the robot pose, so validate
the approach again before a later pen-contact run.

Save drawing parameters and paper/tool setup in project files; keep generated
telemetry in the already ignored `logs/` directory. Bind paper calibration to
robot ID, motor calibration, tool geometry, and base placement. Any relevant
change requires rechecking setup.

## Physical arrangement

- Use a lightweight felt-tip pen or marker with a small compliant holder.
  A fixed, repeatable attachment is preferable to repeatedly changing gripper grip.
- Keep the gripper at one verified holding position throughout a drawing.
- Secure paper and base. The pen must not slip in the holder.
- Start with one roughly 10 mm line; aim for a proposed 20–30 mm square drawing
  patch only after checking reach, loading, and physical accuracy.
- Choose paper height and location so all points can be reached without
  extending the arm. These dimensions are starting proposals, not guaranteed reach.
- Teach a pen-up clearance after measuring physical position uncertainty.
  A nominal 5–10 mm lift is only acceptable if verified to clear the surface.
- Do not change the supply, gains, lead limits, and drawing speed together.

## Mapping a drawing to the robot

Use the official SO101 URDF with the new calibration convention. Verify joint
order, direction, zero conventions, and limits against the saved LeRobot
calibration before sending Cartesian commands. Add an explicit wrist-to-pen-tip
transform; targeting the stock gripper frame would place the wrong point on paper.

Teach three non-collinear pen-tip contact points: paper origin, a point in the
paper X direction, and one in the Y direction. Capture them with torque off and
the arm supported; recording a point must not enable torque. Use their modelled
tip positions to define the paper plane and coordinate axes. Check a fourth point
physically to assess the model/plane error. Three taught points do not, on their
own, solve robot kinematic calibration errors.

Convert each shape into millimetre-scale strokes, map them onto the paper plane,
and insert pen-up, approach, lower, draw, lift, and return segments. Compute inverse
kinematics for the entire path, seeded from the previous solution to discourage
joint flips. Control pen-tip position and its approach axis; leave rotation about
the pen axis free rather than demanding an arbitrary six-dimensional tool pose
from five arm joints. Reject orientation or position errors outside the selected
planning tolerances.

Preflight checks must cover the approach from the current pose, every stroke,
pen-up travels, and the exit. Reject unreachable points, abrupt joint changes,
and calibrated-limit violations. Time the trajectory against both pen speed and
joint speed/acceleration bounds. Begin around 1 mm/s, subject to verification and
the existing conservative joint-speed limit; slow at corners and start/stop smoothly.
Do not skip waypoints or issue catch-up bursts when communication is slow.

No automatic full collision-avoidance guarantee is proposed. Keep execution in a
small, physically cleared workspace, and check link clearance as well as tip
clearance. The published model README notes removed base collision meshes.

## Dependencies and code structure

The current environment has Tkinter and NumPy. Neither SciPy nor Placo is installed.
LeRobot 0.6.1 includes a Placo-based kinematics wrapper, but Placo currently
publishes Linux/macOS wheels. To preserve native Windows/COM3 operation, prefer
a small tested URDF-based forward-kinematics module and bounded inverse
kinematics using SciPy. Verify wheel compatibility with the existing environment
before installing or updating dependencies. Do not upgrade LeRobot as part of this.

Suggested modules for implementation:

- `scripts/drawing_app.py`: UI and explicit user actions.
- `drawing/paths.py`: shapes, paper coordinates, and stroke resampling.
- `drawing/kinematics.py`: model loading, pen transform, forward/inverse kinematics.
- `drawing/planner.py`: complete trajectory generation and validation.
- `drawing/controller.py`: one serial worker, state transitions, telemetry, and stop handling.

Extract the reusable serial/startup/stop logic from the experimental scripts
into a shared module with regression checks. Do not import an entire experimental
test script as the long-term application interface. Robot movements continue to
use LeRobot observations/actions; configuration and readback remain explicit.

## Execution and fault behavior

- Load and preview do not connect to or energize motors.
- Connect leaves torque off. Enable seeds verified current-position hold goals.
- All serial traffic runs in one worker, with the existing UI heartbeat watchdog.
- Pause freezes trajectory advancement and holds verified goals; it does not
  simply disable torque or move the tool to a newly sampled sagged position.
- Resume replans/validates from the measured state with pen clearance checked.
  It must not resume a stale waypoint queue blindly.
- Normal completion lifts the pen only while communication and tracking are
  healthy, then holds a verified end pose. Support before torque-off.
- Faults and emergency torque-off do not attempt an automatic lift or return.
  Keep all-motor stop attempts, individual readback, shutdown lockout, and the
  physical motor-power disconnect option when torque-off cannot be confirmed.
- Maintain electrical, temperature, torque-state, and target-readback checks.
- Assess drawing accuracy in pen-tip millimetres as well as joint errors.
  Warn/abort based on a drawing-specific policy, not silent acceptance of the
  manual UI's 2-degree allowance. Encoder-based tool position is still a model
  estimate, not an independent measurement of contact, compliance, or backlash.
- Log requested path, commanded joints, measured joints, estimated tip, timestamps,
  voltage/current/load/status, and the selected settings for each run.

## Implementation milestones and acceptance checks

| Stage | Deliverable | Check before progressing |
| --- | --- | --- |
| 1. Offline preview | Shape UI, millimetre dimensions, pen-up paths, save/load | Correct geometry and scaling; no serial access |
| 2. Kinematics and setup | Official model, tool transform, paper mapping | FK/IK round trips, joint limits, infeasible-path rejection, continuity, physical convention checks |
| 3. Pen-up motion | One short line traced above paper | Reliable hold/stop, no voltage/communication faults, measured physical tip error and clearance |
| 4. Contact line | One short, lightly drawn line | Light repeatable contact; assess actual line shape and height behavior |
| 5. Shapes | Small square, triangle, then circle | Repeatable strokes, pen lift/travel, and error policy verified in the chosen patch |

Choose an achievable physical accuracy target after stage 3. Start with coarse
shapes; do not promise fine handwriting or submillimetre precision from the
current tracking results. If tip error is too large, improve calibration/control
or reduce scope before increasing pen pressure or drawing size.

## Decisions still needed before contact testing

- Pen attachment and measured tool offset/compliance.
- Reachable paper height/location in the compact workspace.
- Measured pen-up clearance and an achievable drawing-error budget.
- Resolution of recurring voltage/communication faults in that operating region.

## References

- [Official SO101 model and calibration conventions](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md)
- [LeRobot kinematics wrapper](https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/model/kinematics.py)
- [Placo supported wheel platforms](https://pypi.org/project/placo/)
- [SciPy bounded least-squares solver](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
