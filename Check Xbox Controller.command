#!/bin/zsh
cd -- "${0:A:h}" || exit 1
if [[ ! -x xarm-venv/bin/python ]]; then
  print -u2 "Run Setup xArm Xbox.command first."
  exit 1
fi
if [[ ! -x xbox-support/controller_bridge || xbox-support/controller_bridge.swift -nt xbox-support/controller_bridge ]]; then
  xcrun swiftc -O xbox-support/controller_bridge.swift -o xbox-support/controller_bridge || exit 1
fi
./xarm-venv/bin/python -u ./xarm_xbox.py --input-only
