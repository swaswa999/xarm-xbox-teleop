#!/bin/zsh
cd -- "${0:A:h}" || exit 1
if [[ ! -x xarm-venv/bin/python ]]; then
  print -u2 "Run Setup xArm Xbox.command first."
  exit 1
fi
./xarm-venv/bin/python ./xarm_teleop.py --ip "${XARM_IP:-192.168.2.205}"
teleop_exit_status=$?
printf '\nTeleop ended (exit %s). Press any key to close this session.\n' "$teleop_exit_status"
read -k 1
exit "$teleop_exit_status"
