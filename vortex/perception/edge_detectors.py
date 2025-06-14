import cv2
import numpy as np

class LaneKalmanFilter:
    def __init__(self, dt=1.0, process_noise_std=0.1, measurement_noise_std=20.0, initial_estimate_error=100.0):
        self.dt = dt
        self.kalman = cv2.KalmanFilter(8, 4)

        self.kalman.transitionMatrix = np.array([
            [1, 0, 0, 0, dt, 0, 0, 0],
            [0, 1, 0, 0, 0, dt, 0, 0],
            [0, 0, 1, 0, 0, 0, dt, 0],
            [0, 0, 0, 1, 0, 0, 0, dt],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ], np.float32)

        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], np.float32)

        self.kalman.processNoiseCov = np.eye(8, dtype=np.float32) * process_noise_std**2
        self.kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * measurement_noise_std**2
        self.kalman.errorCovPost = np.eye(8, dtype=np.float32) * initial_estimate_error**2

        self.initialized = False

    def predict(self):
        return self.kalman.predict()

    def update(self, measurement):
        measurement = np.array(measurement, np.float32).reshape(4, 1)
        self.kalman.correct(measurement)

    def initialize(self, initial_measurement):
        self.kalman.statePost = np.array([
            initial_measurement[0],
            initial_measurement[1],
            initial_measurement[2],
            initial_measurement[3],
            0.0, 0.0, 0.0, 0.0
        ], np.float32).reshape(8, 1)
        self.initialized = True

    def get_state(self):
        return self.kalman.statePost[:4].flatten().astype(int)

LEFT_KF = LaneKalmanFilter(process_noise_std=0.1, measurement_noise_std=20.0)
RIGHT_KF = LaneKalmanFilter(process_noise_std=0.1, measurement_noise_std=20.0)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def draw_lines(img, lines, color=[255, 0, 0], thickness=5):
    if lines is None:
        return img
    line_img = np.zeros_like(img)
    for line in lines:
        if len(line) == 4:
            x1, y1, x2, y2 = line
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

def extrapolate_lines(img_shape, left_x, left_y, right_x, right_y):
    min_y = int(img_shape[0] * 3 / 5)
    max_y = img_shape[0]

    lines = []

    if left_x and left_y and len(np.unique(left_y)) >= 2:
        try:
            poly_left = np.poly1d(np.polyfit(left_y, left_x, deg=1))
            raw_left_measurement = [int(poly_left(max_y)), max_y, int(poly_left(min_y)), min_y]

            if not LEFT_KF.initialized:
                LEFT_KF.initialize(raw_left_measurement)
            else:
                LEFT_KF.predict()
                LEFT_KF.update(raw_left_measurement)
            lines.append(LEFT_KF.get_state())

        except np.linalg.LinAlgError:
            if LEFT_KF.initialized:
                LEFT_KF.predict()
                lines.append(LEFT_KF.get_state())
    else:
        if LEFT_KF.initialized:
            LEFT_KF.predict()
            lines.append(LEFT_KF.get_state())

    if right_x and right_y and len(np.unique(right_y)) >= 2:
        try:
            poly_right = np.poly1d(np.polyfit(right_y, right_x, deg=1))
            raw_right_measurement = [int(poly_right(max_y)), max_y, int(poly_right(min_y)), min_y]

            if not RIGHT_KF.initialized:
                RIGHT_KF.initialize(raw_right_measurement)
            else:
                RIGHT_KF.predict()
                RIGHT_KF.update(raw_right_measurement)
            lines.append(RIGHT_KF.get_state())

        except np.linalg.LinAlgError:
            if RIGHT_KF.initialized:
                RIGHT_KF.predict()
                lines.append(RIGHT_KF.get_state())
    else:
        if RIGHT_KF.initialized:
            RIGHT_KF.predict()
            lines.append(RIGHT_KF.get_state())

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

    extrapolated = None
    line_image = image

    left_x, left_y, right_x, right_y = separate_lines(lines, image.shape) if lines is not None else ([], [], [], [])
    extrapolated = extrapolate_lines(image.shape, left_x, left_y, right_x, right_y)

    if extrapolated:
        line_image = draw_lines(image, extrapolated[0])
    else:
        line_image = image

    return extrapolated, line_image
