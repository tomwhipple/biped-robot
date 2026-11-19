"""Shared tag definitions: ids, sizes, roles; AprilTag 36h11 rendering and detection."""
import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# id -> (size_mm, role). Size is the outer edge of the black border.
TAGS = {
    0: (80, "board far-left corner"),
    1: (80, "board far-right corner"),
    2: (80, "board near-left corner"),
    3: (80, "board near-right corner"),
    4: (80, "board mid-edge, left side (optional)"),
    5: (80, "board mid-edge, right side (optional)"),
    10: (60, "torso front"),
    11: (60, "torso back"),
    12: (60, "torso top (faces up)"),
    13: (25, "left foot top (optional)"),
    14: (25, "right foot top (optional)"),
}
BOARD_IDS = (0, 1, 2, 3, 4, 5)
ROBOT_IDS = (10, 11, 12, 13, 14)


def tag_image(tag_id, px):
    """px x px uint8 image of the tag (8x8 modules incl. the black border)."""
    return cv2.aruco.generateImageMarker(DICT, tag_id, px, borderBits=1)


def make_detector():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    p.minMarkerPerimeterRate = 0.02
    p.adaptiveThreshWinSizeMax = 63
    p.adaptiveThreshWinSizeStep = 6
    p.errorCorrectionRate = 0.6
    # measured on synthetic renders (tests/test_vision_tags.py): these bit-sampling
    # settings plus a 2x upscale are what lets 16-20 px tags decode
    p.perspectiveRemovePixelPerCell = 8
    p.perspectiveRemoveIgnoredMarginPerCell = 0.2
    return cv2.aruco.ArucoDetector(DICT, p)


def detect(gray, detector=None, upscale=None):
    """-> {id: 4x2 float64 corners (TL,TR,BR,BL in the tag's own frame)}, pixels of `gray`."""
    detector = detector or make_detector()
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    if upscale is None:       # small frames (Sony 1024 px): tags of 16-30 px need the 2x; 1080p tags are 45+ px
        upscale = 2 if gray.shape[1] < 1400 else 1
    if upscale != 1:
        gray = cv2.resize(gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
    corners, ids, _ = detector.detectMarkers(gray)
    out = {}
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            out[int(i)] = c.reshape(4, 2).astype(np.float64) / upscale
    return out
