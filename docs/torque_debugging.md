# SO101 torque and tracking review

## Current evidence

- Environment: LeRobot 0.6.1, Feetech servo SDK 1.0.0, COM3,
  calibration ID `so101_follower_TCH`.
- Reported hardware: STS3215 C001 motors, 5 V / 4 A supply, Waveshare
  Bus Servo Adapter (A), both communication jumpers in B for USB.
- Several loaded joints stall, while compact poses can pass. Stored goals
  match commands and torque remains enabled. This is not specific to one joint.
- Earlier elbow inspection found `Max_Torque_Limit=1000`, `Torque_Limit=1000`,
  `Protection_Current=500`. These are historical readings, not a fresh audit of
  all motors or a measurement at the time of each failure.
- Typical reported stalls: position error around 2.2–2.4 degrees, load counts
  around 240–250, current counts around 23–34, voltage counts 53–55, status 0.
- A separate gripper voltage fault occurred once and subsequently cleared.
  Stable sampled voltage does not exclude short transients at other motors.
- Later posted failures confirm P=40 and P=48 were applied. At P=48 the user
  reports substantially improved movement in extended poses, with a remaining
  full-extension stall: error 2.438 degrees, load 340, current 57, voltage 53,
  both torque ceilings 1000, status 0. This supports controller effort as a
  contributor without establishing maximum motor torque or a supply current cap.

## What the controls mean

The read-only audit in `logs/motor_audit.json` (2026-11-16 17:23 UTC) now confirms:

- All five body motors: max torque 1000, runtime torque 1000, protection current
  500, protective torque 20, overload torque 80.
- Gripper: max/runtime torque 500, protection current 250, overload torque 25,
  consistent with the deliberately reduced gripper settings.
- All motors: P=16, I=0, D=32, position mode 0, firmware 3.10, status 0,
  torque off, and successful communication without packet faults.
- At rest, voltage is 5.4–5.5 V and temperature is 20–21 C.

P=16 at rest is expected after the temporary UI overrides are restored. This
audit finds no reduced body-joint torque ceiling to raise. It does not capture
runtime protection behavior during a stall. The updated UI failure report
should be used to obtain that comparison before changing protection settings.

Subsequent failure output confirms both torque limits remain 1000 on the elbow
while enabled and stalled, with status 0. Two failures have `desired=commanded`
and final position errors of 1.560 and 1.778 degrees. In these examples the
2.5-degree command-lead limit is not active: the requested destination is already
stored in the servo. The UI's 1.5-degree tracking threshold declares a failure
after three seconds of little progress. Load-dependent steady position offset
with I=0 is a hypothesis to investigate; these examples do not establish a
current-protection event or maximum motor output.

The same report includes an intermittent gripper voltage fault and a corrupt
reply during torque-off on ID 4. Resolve electrical/communication reliability
before further loaded movement tests. UI shutdown now attempts all six motors
independently with bounded retries and individual torque readback. If any motor's
OFF state cannot be confirmed, gains are not restored and enabling is locked out
until the application is restarted after inspection. The user must support the
arm and disconnect motor power when torque-off cannot be confirmed. Closing the
serial port is not equivalent to removing motor power.

| Setting | Purpose | Consequence for debugging |
| --- | --- | --- |
| `Max_Torque_Limit` / `Torque_Limit` | Stored and runtime output ceilings, on a 0–1000 scale | If both are already 1000, there is no additional ceiling to unlock by writing a higher number. |
| `Protection_Current` | Overcurrent protection threshold | Raising it does not command more current; identify an actual protection event before considering changes. |
| `Protective_Torque`, `Overload_Torque`, `Protection_Time` | Overload protection behavior | Distinguish these from the normal output ceiling; inspect them rather than disabling protection. |
| P/I/D gains | How position error becomes motor drive | Higher P can develop more effort at a given error. I can address persistent error but requires separate tuning and attention to windup. |

Feetech describes load feedback as motor-drive duty cycle, with 1000 full scale,
and current feedback as 6.5 mA per count. Thus load=241 is about 24.1% drive and
current=34 is about 0.221 A of reported servo current. Neither is a calibrated
measurement of joint torque, and servo current is not total supply current.
These readings do not demonstrate maximum motor output.

The UI deliberately holds targets within about 2.5 degrees of measured body
positions; LeRobot is also configured with `max_relative_target=3.0`.
Those are our script's limits, not servo factory limits. With I=0, they can
prevent the position error needed to generate sufficient effort against gravity.
The same numeric UI thresholds apply to the gripper's 0–100 units, not degrees.

The UI now offers isolated opt-in `--shoulder-lead 3.0` and `--elbow-lead 3.0`
comparisons. Each default remains 2.5 degrees. Only a selected joint's lead
changes; its LeRobot backup relative-target cap is lead + 0.5 degrees. Other
joints retain their default caps, and the 5-unit error cutoff, progress watchdog,
temperature/fault checks, calibration bounds, and speed limits remain active.
The proposed next comparison keeps shoulder P=48 and increases only shoulder
lead to 3.0 degrees. It has not yet been physically tested. A larger permitted
position error can produce more corrective effort and stored mechanical energy;
this experiment does not change torque ceilings or current protection.

Startup configuration follows LeRobot's settings: body P=16, I=0, D=32 before
explicit overrides. Gripper protection is deliberately reduced to max torque
500, protection current 250, overload torque 25. This is gripper-specific and
must not be mistaken for a body-joint output limit. The UI defaults to shoulder
P=32 and elbow P=36 while enabled, restoring the pre-override values on stop.

## Next diagnostic sequence

1. Support the arm and close the UI so its serial connection is released.
2. Capture settings without invoking robot configuration or changing torque:

   ```powershell
   .\.venv\Scripts\python.exe scripts\inspect_motor.py --port COM3 --all --output logs\motor_audit.json
   ```

   This reads all six motors and saves raw values, communication failures, and
   packet fault flags separately. It sends no goal or configuration writes.

3. Check both torque ceilings on each body joint. A genuinely reduced ceiling
   is a candidate for a temporary, verified change within the documented range.
   Do not raise current protection simply because a joint stopped moving.
4. If settings are normal, reproduce one short movement at fixed pose, speed,
   and gain. The UI now includes torque ceilings and protection settings in its
   failure output, before gain restoration. This tests whether the values differ
   during a failure from the at-rest audit.
5. If ceilings are full but drive stays modest, investigate the position
   controller and bounded target lead rather than assuming a supply current cap.
   Change one parameter at a time. Do not simultaneously raise gains, position
   error allowance, speed, and protection thresholds.
6. If motor voltage drops or a voltage fault reappears, inspect power delivery
   at the affected servo as well as at the board. Motor-terminal measurements
   require safely accessible contacts and protection against accidental shorts.

For the C001 motor, the manufacturer datasheet specifies a 4–7.4 V input range
and torque ratings at 6 V and 7.4 V. The project BOM intentionally uses 5 V;
that is not evidence of an incorrect build. A regulated supply at a higher
supported voltage can increase available torque, but is a separate hardware
change requiring compatible polarity, connectors, current capacity, and every
attached device's voltage rating. Do not apply 12 V to the C001 motors.

## Sources

- [Feetech load/current feedback and protections](https://ftservo.com/www.feetech.cn)
- [Manufacturer C001 datasheet](https://files.seeedstudio.com/products/Feetech/108090023_STS3215-C001_Datasheet.pdf)
- [LeRobot 0.6.1 follower configuration](https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/robots/so_follower/so_follower.py)
- [SO101 bill of materials](https://github.com/TheRobotStudio/SO-ARM100#sourcing-parts)
- [Waveshare adapter specifications](https://docs.waveshare.com/Bus_Servo_Adapter_A)

This review and its tests did not connect to or move the physical robot.
