# Third-party notices

## UFACTORY teleoperation examples

The mode-7 Xbox jogging approach was adapted from:

- Repository: https://github.com/xArm-Developer/ufactory_teleop
- Reference revision: `d149f5a5a4e7e8b6d82561fd64bd0dfd240bf5bf`
- Reference file: `xbox_teleop/uf_robot_xbox_teleop.py`
- Copyright: 2025, UFACTORY Inc.
- License: BSD 3-Clause; the complete notice is retained in
  [LICENSES/ufactory-teleop-BSD-3-Clause.txt](LICENSES/ufactory-teleop-BSD-3-Clause.txt).

The reference repository is not a runtime dependency and is not bundled in this
repository. Its `UFRobot` startup wrapper is not invoked.

## Installed dependencies

The setup script installs `xarm-python-sdk==1.18.4` and `pygame-ce==2.5.8`.
Their source distributions and installed packages carry their own licenses:

- https://github.com/xArm-Developer/xArm-Python-SDK
- https://github.com/pygame-community/pygame-ce

Python environments and compiled helper binaries are not committed.
