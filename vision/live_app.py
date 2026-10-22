import base64
import json
from pathlib import Path

import cv2
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from starlette.concurrency import run_in_threadpool
from starlette.middleware.trustedhost import TrustedHostMiddleware

from live_tracker import LiveTracker

cv2.setNumThreads(4)
app = FastAPI(title="Live wheel tracking")
app.add_middleware(TrustedHostMiddleware, allowed_hosts=["localhost","127.0.0.1","[::1]","testserver"])


@app.get("/")
def index():
    return FileResponse(Path(__file__).parent / "live.html")


@app.get("/health")
def health():
    return {"ok":True,"mode":"live tracking with manual initialization"}


@app.get("/sample")
def sample():
    from fastapi import HTTPException
    path = Path(__file__).parent / "video-output" / "working.mp4"
    if not path.is_file():
        raise HTTPException(404,"Sample video is unavailable.")
    return FileResponse(path,media_type="video/mp4")


def decode(data):
    if len(data) > 2_000_000:
        raise ValueError("Camera frame is too large.")
    frame = cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
    if frame is None or max(frame.shape[:2]) > 1280:
        raise ValueError("Invalid camera frame. Maximum dimension is 1280 pixels.")
    return frame


@app.websocket("/track")
async def track(socket: WebSocket):
    origin = socket.headers.get("origin")
    if origin != f"http://{socket.headers.get('host')}":
        await socket.close(code=1008)
        return
    await socket.accept()
    tracker = None
    try:
        while True:
            message = await socket.receive()
            if message["type"] == "websocket.disconnect":
                break
            try:
                if message.get("text") is not None:
                    if len(message["text"]) > 3_000_000:
                        raise ValueError("Initialization frame is too large.")
                    payload = json.loads(message["text"])
                    if payload.get("type") != "initialize":
                        raise ValueError("Unknown request.")
                    frame = decode(base64.b64decode(payload["frame"],validate=True))
                    tracker = await run_in_threadpool(LiveTracker,frame,payload["boxes"])
                    result = tracker.result()
                else:
                    if tracker is None:
                        raise ValueError("Select the tire and lug nuts first.")
                    frame = decode(message["bytes"])
                    result = await run_in_threadpool(tracker.update,frame)
                await socket.send_json(result)
            except (ValueError, KeyError, TypeError, cv2.error) as error:
                await socket.send_json({"status":"error","message":str(error)[:250],"boxes":[]})
    except WebSocketDisconnect:
        pass
