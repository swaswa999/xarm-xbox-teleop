# Lugnut Tracker

A local app for tracking a tire and its lug nuts in webcam, USB camera, or uploaded video footage. Select the objects once, then follow their movement with bounding boxes and tracking statistics.

**Manual initialization is required.** This version uses OpenCV feature matching and optical flow. It does not automatically recognize lug nuts. Confidence measures geometric tracking quality, not object-recognition probability.

## Run

Requires Python 3.12 and a browser with camera access.

On macOS, open **Start Live Tracking.command**. It creates a virtual environment and installs dependencies on first launch. Keep its terminal window open while using the app.

Or run from the project directory:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-live.txt
.venv/bin/python -m uvicorn live_app:app --host 127.0.0.1 --port 8766 --ws-max-size 4000000
```

Open http://localhost:8766.

1. Enable camera access and choose the built-in webcam or a USB camera. Use **Refresh list** after connecting a device.
2. Alternatively, use **Open video file** to load a browser-compatible local video.
3. Click **Freeze & select**, box the entire tire first, then box each visible lug nut.
4. Click **Start tracking**.

Switching cameras clears selections. Select the objects again in the new view.

## Features

- Camera selection, device refresh, and disconnect handling
- Stable object IDs and bounding boxes
- Tracking confidence, FPS, latency, feature agreement, and alignment diagnostics
- Annotated snapshots and silent recordings
- CSV coordinates and JSON session exports
- Local processing without a cloud API or API key

See [LIVE-README.md](LIVE-README.md) for metric definitions, limitations, and export details. Original footage and generated recordings are excluded from this repository. The optional **Load sample video** feature requires the original clip at `video-output/working.mp4`; ordinary video upload works without it.

## Checks

```sh
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q
node --test tests/camera-controls.test.cjs
```

Python tests exercise motion, loss/recovery, input validation, and WebSocket behavior. Node tests use mocked cameras to exercise switching, unplugging, permission errors, and cancellation. Physical camera behavior depends on browser permissions and hardware.
