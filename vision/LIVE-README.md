# Live wheel tracker

Open `Start Live Tracking.command`, then visit http://localhost:8766 in your browser. Keep the terminal window open while using the app.

## Camera setup

1. Connect your USB camera, or use the Mac webcam.
2. Click **Enable camera** and allow the browser to access your camera. The **Camera device** menu lists the built-in webcam and connected USB cameras by name. Select another device to switch cameras immediately. Use **Refresh list** after connecting a camera if it has not appeared automatically. Switching stops tracking and recording and clears the old selections; select the objects again in the new view. When no camera is connected, choose a device and click **Connect selected camera**.
3. Frame a well-lit wheel with the lug nuts clearly visible.
4. Click **Freeze & select**. Drag a box around the tire first, then each lug nut. Use **Undo box** to correct selections.
5. Click **Start tracking**. Use **Reset** to select another wheel.

The camera sends JPEG frames to the local Python process on this Mac. No cloud inference or API key is used. Camera video is not saved unless you choose to record or save a snapshot.

**Load sample video** opens the original wheel clip with six initial selections if it is available at `video-output/working.mp4`. The clip is not included in this repository. **Open video file** supports other browser-playable videos with manual selections.

## Statistics

- Tracking confidence (0–100): a geometric quality heuristic shared by the wheel's tracks. It is not a calibrated recognition probability or a claim that a nut is present.
- Matched features: correspondences used to estimate motion.
- Inlier agreement: fraction of correspondences agreeing with the estimated transformation.
- Alignment error: median feature reprojection error in analysis pixels.
- Analysis rate: observed completed frames per second.
- Round-trip latency: JPEG upload, processing, and response time; excludes camera sensor/display latency.
- Processing time: tracker time on the Python server.
- Object tracks: stable IDs, center position, and bounding box dimensions.
- Tracking lost: transitions into a state where reliable wheel motion could not be estimated.

Confidence formula: `100 × min(1, matched_features / 60) × inlier_ratio × exp(−median_alignment_error / 5)`.

## Outputs

**Save snapshot** downloads the displayed image and boxes. **Record** records the displayed canvas; stop recording, then click the generated download link. Recordings are silent and their displayed frame rate follows available tracking updates.

**Download box data · CSV** exports per-frame object coordinates and metrics. **Download session · JSON** exports initial selections, definitions, summary, and frame data. Export before reloading the page or starting another session. Data is held in browser memory.

## Limits and validation

This version tracks objects you identify once. It does not automatically identify new lug nuts, verify that a nut is present, or detect removals. The tracker assumes the selected objects move together with the wheel. If individual parts move independently, reset their selections. Strong motion blur, occlusion, rapid camera movement, or a large viewpoint change can break tracking; lost boxes are hidden, with recovery attempted against the original frame.

The tracker resizes the longest image dimension to 960 pixels. Small objects may require a closer camera view. The analysis-rate slider sets a target; it does not guarantee that rate.

Tested on 151 frames sampled at 15 fps from the supplied clip: no lost frames, mean tracker time 13.2 ms, p95 36.8 ms on this Mac. Median lug center difference from the reviewed offline tracks was 0.67 pixels at 540×960 resolution; maximum 5.5 pixels. This comparison is not independent ground-truth accuracy. Physical camera performance still needs to be checked with your camera and lighting.

Python tests cover motion tracking, tracking loss/recovery, invalid selections/dimensions, and WebSocket initialization/origin checks. Mock camera tests cover camera switching, unplugging, permission failures, and cancellation.

## Installation and development

Use Python 3.12 and install `requirements-live.txt` in a virtual environment:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-live.txt
.venv/bin/python -m uvicorn live_app:app --host 127.0.0.1 --port 8766 --ws-max-size 4000000
```

Install `requirements-dev.txt` to run the automated Python tests. Camera-control tests run with `node --test tests/camera-controls.test.cjs`.
