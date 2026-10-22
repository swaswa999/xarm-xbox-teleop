"""Camera-independent tracker; boxes are selected once on a reference frame."""
import time

import cv2
import numpy as np


def project(points, matrix):
    return cv2.perspectiveTransform(np.float32(points).reshape(-1, 1, 2), matrix).reshape(-1, 2)


class LiveTracker:
    def __init__(self, frame, boxes):
        self.height, self.width = frame.shape[:2]
        self.reference = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.previous = self.reference
        self.matrix = np.eye(3)
        self.index = 0
        self.lost = False
        self.metrics = {"confidence":100.0,"inlier_ratio":1.0,"reprojection_error_px":0.0,"features":len(boxes)}
        self.boxes = []
        if not 2 <= len(boxes) <= 16:
            raise ValueError("Select the tire and at least one lug nut.")
        for index, item in enumerate(boxes):
            xyxy = np.asarray(item["xyxy"], dtype=np.float32)
            if xyxy.shape != (4,) or not np.isfinite(xyxy).all():
                raise ValueError("Invalid selection.")
            x1, y1, x2, y2 = xyxy
            if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
                raise ValueError("Selections must be inside the image.")
            pixels = xyxy * [self.width, self.height, self.width, self.height]
            x1, y1, x2, y2 = pixels
            if min(x2-x1, y2-y1) < 6:
                raise ValueError("Make each selection at least 6 pixels wide and tall.")
            self.boxes.append({"id": "T1" if index == 0 else f"L{index}",
                               "label": "tire" if index == 0 else "lug nut",
                               "points": np.float32([[x1,y1],[x2,y1],[x2,y2],[x1,y2]])})
        tire = self.boxes[0]["points"]
        center = tuple(np.round(tire.mean(axis=0)).astype(int))
        axes = tuple(np.maximum(8, (tire[2]-tire[0])*.36).astype(int))
        self.mask = np.zeros_like(self.reference)
        cv2.ellipse(self.mask, center, axes, 0, 0, 360, 255, -1)
        self.sift = cv2.SIFT_create(nfeatures=1200, contrastThreshold=.015)
        self.keypoints, self.descriptors = self.sift.detectAndCompute(self.reference, self.mask)
        if self.descriptors is None or len(self.keypoints) < 20:
            raise ValueError("Too little wheel detail. Move closer, improve the light, then select again.")
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.anchors = np.float32([b["points"].mean(axis=0) for b in self.boxes[1:]])

    def direct_match(self, gray):
        keypoints, descriptors = self.sift.detectAndCompute(gray, None)
        if descriptors is None:
            return None
        pairs = self.matcher.knnMatch(self.descriptors, descriptors, k=2)
        good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < .7*p[1].distance]
        if len(good) < 20:
            return None
        source = np.float32([self.keypoints[m.queryIdx].pt for m in good])
        target = np.float32([keypoints[m.trainIdx].pt for m in good])
        matrix, mask = cv2.findHomography(source, target, cv2.USAC_MAGSAC, 2.5)
        if matrix is None or int(mask.sum()) < 20:
            return None
        keep = mask.ravel().astype(bool)
        error = float(np.median(np.linalg.norm(project(source[keep],matrix)-target[keep],axis=1)))
        return matrix, int(mask.sum()), float(keep.mean()), error

    def plausible(self, matrix):
        if matrix is None or not np.isfinite(matrix).all():
            return False
        original = self.boxes[0]["points"]
        transformed = project(original, matrix)
        area = cv2.contourArea(transformed)
        reference_area = cv2.contourArea(original)
        if not (.08 < area/reference_area < 12):
            return False
        return bool(cv2.isContourConvex(transformed))

    def update(self, frame):
        started = time.perf_counter()
        if frame.shape[:2] != (self.height, self.width):
            raise ValueError("Camera dimensions changed. Reset the selections.")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidate = None
        features = 0
        ratio = 0.0
        residual = 10.0
        if not self.lost:
            mask = cv2.warpPerspective(self.mask, self.matrix, (self.width, self.height))
            points = cv2.goodFeaturesToTrack(self.previous, 250, .01, 7, mask=mask, blockSize=7)
            if points is not None and len(points) >= 20:
                moved, status, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, points, None,
                                                          winSize=(31,31), maxLevel=4)
                back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, moved, None,
                                                              winSize=(31,31), maxLevel=4)
                valid = (status.ravel() == 1) & (back_status.ravel() == 1) & (
                    np.linalg.norm(points.reshape(-1,2)-back.reshape(-1,2), axis=1) < 1.5)
                if valid.sum() >= 20:
                    step, inliers = cv2.findHomography(points[valid], moved[valid], cv2.USAC_MAGSAC, 2.5)
                    if step is not None and int(inliers.sum()) >= 20:
                        candidate = step @ self.matrix
                        features = int(inliers.sum())
                        keep = inliers.ravel().astype(bool)
                        ratio = float(keep.mean())
                        residual = float(np.median(np.linalg.norm(project(points[valid][keep],step)-moved[valid][keep].reshape(-1,2),axis=1)))
        if candidate is None or self.index % 12 == 0:
            direct = self.direct_match(gray)
            if direct and self.plausible(direct[0]):
                if candidate is None or np.max(np.linalg.norm(
                        project(self.anchors, candidate)-project(self.anchors, direct[0]), axis=1)) < 20:
                    candidate, features, ratio, residual = direct
        self.previous = gray
        self.index += 1
        if not self.plausible(candidate):
            self.lost = True
            self.metrics = {"confidence":0.0,"inlier_ratio":0.0,"reprojection_error_px":None,"features":0}
            return {"status": "lost", "boxes": [], "message": "Tracking lost. Hold the original wheel in view or reset.",
                    **self.metrics,
                    "processing_ms": round((time.perf_counter()-started)*1000,1)}
        self.matrix = candidate / candidate[2,2]
        self.lost = False
        # Geometric tracking quality, not a calibrated recognition probability.
        confidence = 100 * min(1,features/60) * ratio * np.exp(-residual/5)
        self.metrics = {"confidence":round(float(confidence),1),"inlier_ratio":round(ratio,3),
                        "reprojection_error_px":round(residual,3),"features":features}
        return self.result(features, started)

    def result(self, features=0, started=None):
        results = []
        for box in self.boxes:
            points = project(box["points"], self.matrix)
            bounds = np.r_[points.min(axis=0),points.max(axis=0)]
            bounds /= [self.width,self.height,self.width,self.height]
            # Hide objects completely outside the view instead of pinning them to an edge.
            if bounds[2] <= 0 or bounds[3] <= 0 or bounds[0] >= 1 or bounds[1] >= 1:
                continue
            results.append({"id":box["id"],"label":box["label"],"xyxy":bounds.clip(0,1).tolist()})
        return {"status":"tracking","boxes":results,**self.metrics,
                "processing_ms":round((time.perf_counter()-started)*1000,1) if started else 0}
