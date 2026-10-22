import base64
import json

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from live_app import app
from live_tracker import LiveTracker


def fixture_frame():
    rng = np.random.default_rng(4)
    frame = rng.integers(0,256,(300,400,3),dtype=np.uint8)
    return cv2.GaussianBlur(frame,(3,3),0)


BOXES = [{"xyxy":[.1,.1,.9,.9]},{"xyxy":[.45,.45,.55,.55]}]


def test_tracks_translation_and_recovers_after_loss():
    frame = fixture_frame()
    tracker = LiveTracker(frame,BOXES)
    translated = cv2.warpAffine(frame,np.float32([[1,0,8],[0,1,5]]),(400,300))
    result = tracker.update(translated)
    assert result["status"] == "tracking"
    assert result["features"] >= 20
    assert 0 < result["confidence"] <= 100
    np.testing.assert_allclose(result["boxes"][1]["xyxy"],[.47,.45+5/300,.57,.55+5/300],atol=.01)
    result = tracker.update(np.zeros_like(frame))
    assert result["status"] == "lost" and result["boxes"] == []
    assert tracker.update(frame)["status"] == "tracking"


def test_requires_valid_boxes_and_same_dimensions():
    with pytest.raises(ValueError):
        LiveTracker(fixture_frame(),[{"xyxy":[0,0,1,1]}])
    with pytest.raises(ValueError):
        LiveTracker(fixture_frame(),[{"xyxy":[0,0,1,1]},{"xyxy":[.7,.7,.2,.2]}])
    tracker = LiveTracker(fixture_frame(),BOXES)
    with pytest.raises(ValueError):
        tracker.update(np.zeros((200,200,3),dtype=np.uint8))


def test_websocket_initialization_and_binary_frames():
    frame = fixture_frame()
    _,jpeg = cv2.imencode('.jpg',frame)
    with TestClient(app) as client:
        assert client.get('/').status_code == 200
        with client.websocket_connect('/track',headers={"origin":"http://testserver"}) as socket:
            socket.send_json({"type":"initialize","boxes":BOXES,"frame":base64.b64encode(jpeg).decode()})
            result = socket.receive_json()
            assert result['status'] == 'tracking' and len(result['boxes']) == 2
            socket.send_bytes(jpeg.tobytes())
            assert socket.receive_json()['status'] == 'tracking'


def test_rejects_cross_origin_websockets():
    from starlette.websockets import WebSocketDisconnect
    with TestClient(app) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect('/track',headers={"origin":"https://example.org"}):
                pass
