#!/bin/zsh
set -e
cd "${0:A:h}"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
  .venv/bin/python -m pip install -r requirements-live.txt
fi
if curl -fsS http://127.0.0.1:8766/health >/dev/null 2>&1; then
  open http://127.0.0.1:8766
  exit 0
fi
(sleep 2; open http://127.0.0.1:8766) &
exec .venv/bin/python -m uvicorn live_app:app --host 127.0.0.1 --port 8766 --ws-max-size 4000000
