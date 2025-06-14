import cv2
import numpy as np
from collections import deque

LEFT_HISTORY = deque(maxlen=10)
RIGHT_HISTORY = deque(maxlen=10)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def draw_lines(img, lines, color=[255, 0, 0], thickness=5):
    if lines is None:
        return img
    line_img = np.zeros_like(img)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    return cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

def separate_lines(lines, img_shape):
    left_x, left_y = [], []
    right_x, right_y = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue  
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.2:
                continue
            if slope < 0:
                left_x += [x1, x2]
                left_y += [y1, y2]
            else:
                right_x += [x1, x2]
                right_y += [y1, y2]
    return left_x, left_y, right_x, right_y

def smooth_line(history, new_line):
    if new_line is not None:
        history.append(new_line)
    if history:
        return np.mean(history, axis=0).astype(int)
    return None

def extrapolate_lines(img_shape, left_x, left_y, right_x, right_y):
    min_y = int(img_shape[0] * 3 / 5)
    max_y = img_shape[0]

    lines = []

    if left_x and left_y:
        poly_left = np.poly1d(np.polyfit(left_y, left_x, deg=1))
        raw_left = [int(poly_left(max_y)), max_y, int(poly_left(min_y)), min_y]
        smoothed_left = smooth_line(LEFT_HISTORY, raw_left)
        if smoothed_left is not None:
            lines.append(smoothed_left)

    if right_x and right_y:
        poly_right = np.poly1d(np.polyfit(right_y, right_x, deg=1))
        raw_right = [int(poly_right(max_y)), max_y, int(poly_right(min_y)), min_y]
        smoothed_right = smooth_line(RIGHT_HISTORY, raw_right)
        if smoothed_right is not None:
            lines.append(smoothed_right)

    return [lines] if lines else None

def process_image(image):
    height, width = image.shape[:2]

    top = int(height * 0.4)
    bottom = height
    roi_vertices = np.array([[
        (0, bottom),
        (0, top),
        (width, top),
        (width, bottom)
    ]], dtype=np.int32)

    blurred = cv2.GaussianBlur(image, (3, 3), 0)
    edges = cv2.Canny(blurred, 100, 200)
    cropped_edges = region_of_interest(edges, roi_vertices)

    lines = cv2.HoughLinesP(
        cropped_edges,
        rho=6,
        theta=np.pi / 60,
        threshold=160,
        lines=np.array([]),
        minLineLength=40,
        maxLineGap=25
    )

    if lines is not None:
        left_x, left_y, right_x, right_y = separate_lines(lines, image.shape)
        extrapolated = extrapolate_lines(image.shape, left_x, left_y, right_x, right_y)
        line_image = draw_lines(image, extrapolated)
    else:
        line_image = image

    return extrapolated, line_image
