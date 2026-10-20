#!/bin/zsh
set -euo pipefail
cd -- "${0:A:h}"

python_executable="${PYTHON:-python3.12}"
if ! command -v "$python_executable" >/dev/null 2>&1; then
  print -u2 "Install Python 3.12, or set PYTHON to its executable path."
  exit 1
fi
"$python_executable" -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 12) else "Please use Python 3.12 for this setup.")'

if [[ "$(uname -s)" != Darwin ]] || ! xcrun --find swiftc >/dev/null 2>&1; then
  print -u2 "This setup needs macOS and Apple's Command Line Tools."
  print -u2 "Install the tools with: xcode-select --install"
  exit 1
fi

"$python_executable" -m venv xarm-venv
./xarm-venv/bin/python -m pip install -r requirements.txt
xcrun swiftc -O xbox-support/controller_bridge.swift -o xbox-support/controller_bridge
print "Setup complete. Open Check Xbox Controller.command to test inputs without robot access."
