#!/bin/zsh
cd -- "${0:A:h}" || exit 1
if [[ ! -x xarm-venv/bin/python ]]; then
  print -u2 "Run Setup xArm Xbox.command first."
  exit 1
fi
if [[ ! -x xbox-support/controller_bridge || xbox-support/controller_bridge.swift -nt xbox-support/controller_bridge ]]; then
  xcrun swiftc -O xbox-support/controller_bridge.swift -o xbox-support/controller_bridge || exit 1
fi
./xarm-venv/bin/python -u ./xarm_xbox.py --ip "${XARM_IP:-192.168.2.205}"
xbox_exit_status=$?
printf '\nXbox controls ended (exit %s). Press any key to close.\n' "$xbox_exit_status"
read -k 1
exit "$xbox_exit_status"
