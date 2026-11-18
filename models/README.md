# SO101 kinematic model

`so101_new_calib.urdf` is the unmodified official RobotStudio SO101 model,
downloaded on 2026-11-18 from:

https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf

SHA-256:
`3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`

Upstream: https://github.com/TheRobotStudio/SO-ARM100

The upstream Apache 2.0 license is included as `LICENSE`. The XYZ test reads
joint transforms and limits only, so the mesh files referenced by the model are
not required and are not included. It does not perform mesh collision checking.

The model uses the newer middle-of-range joint-zero convention. Agreement with
the particular physical motor calibration must still be checked. Its default
`gripper_frame_link` reference is not automatically the tip of an attached pen.
Specify a local XYZ tool offset in millimetres when a tool is fitted.
