import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Constants
TRIG = 17
ECHO = 27
LED_PIN = 22

# Motor Pins
MOTOR1_FORWARD = 16
MOTOR1_BACKWARD = 12
MOTOR2_FORWARD = 21
MOTOR2_BACKWARD = 20

# Motor Speed (PWM Frequency)
SPEED = 100
OBSTACLE_THRESHOLD = 25
MOTOR_SPEED = 80
TURN_SPEED = 83
STEERING_SMOOTHING_FACTOR = 0.2

# Frame dimensions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Setup GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)

GPIO.setup(MOTOR1_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR1_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR2_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR2_BACKWARD, GPIO.OUT)

# Setup PWM
motor1_forward = GPIO.PWM(MOTOR1_FORWARD, SPEED)
motor1_backward = GPIO.PWM(MOTOR1_BACKWARD, SPEED)
motor2_forward = GPIO.PWM(MOTOR2_FORWARD, SPEED)
motor2_backward = GPIO.PWM(MOTOR2_BACKWARD, SPEED)

motor1_forward.start(0)
motor1_backward.start(0)
motor2_forward.start(0)
motor2_backward.start(0)

GPIO.output(LED_PIN, True)
time.sleep(2)


def measure_distance():
    try:
        GPIO.output(TRIG, False)
        time.sleep(0.1)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = pulse_end = time.time()

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)
    except Exception as e:
        logger.error(f"Distance measurement error: {e}")
        return None


def stop():
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(0)
    logger.info('Stopped')


def forward(speed=MOTOR_SPEED):
    logger.info('Moving Forward')
    motor1_forward.ChangeDutyCycle(speed)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(speed)
    motor2_backward.ChangeDutyCycle(0)


def backward(speed=MOTOR_SPEED):
    logger.info('Moving Backward')
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(speed)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(speed)


def turn_left(speed=TURN_SPEED):
    logger.info('Turning Left')
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(speed)
    motor2_backward.ChangeDutyCycle(0)


def turn_right(speed=TURN_SPEED):
    logger.info('Turning Right')
    motor1_forward.ChangeDutyCycle(speed)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(0)


def detect_yellow_lane(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 70, 70])
    upper_yellow = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_lane = cv2.bitwise_and(frame, frame, mask=mask)
    return yellow_lane, mask


def detect_lane_edges(mask):
    blur = cv2.GaussianBlur(mask, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (0, int(height * 0.6)),
        (width, int(height * 0.6)),
        (width, height)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges


def average_slope_intercept(lines):
    left_lines = []
    right_lines = []
    left_weights = []
    right_weights = []

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 - x1 == 0:  # Avoid division by zero
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

                if slope < 0:  # Left lane
                    left_lines.append((slope, intercept))
                    left_weights.append(length)
                else:  # Right lane
                    right_lines.append((slope, intercept))
                    right_weights.append(length)

    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

    return left_lane, right_lane


def make_coordinates(y1, y2, line):
    if line is None:
        return None
    slope, intercept = line
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def compute_steering_direction(lines, frame_shape):
    height, width, _ = frame_shape
    left_lane, right_lane = average_slope_intercept(lines)

    y1 = height
    y2 = int(height * 0.6)

    left_line = make_coordinates(y1, y2, left_lane)
    right_line = make_coordinates(y1, y2, right_lane)

    if left_line is not None and right_line is not None:
        mid = width // 2
        lane_center = (left_line[2] + right_line[2]) // 2
        offset = lane_center - mid

        if abs(offset) < width * STEERING_SMOOTHING_FACTOR:
            return "forward"
        elif offset > 0:
            return "right"
        else:
            return "left"
    elif left_line is not None:
        return "left"
    elif right_line is not None:
        return "right"
    else:
        return "stop"


def draw_lines(frame, lines):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
    return line_image


try:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("Camera could not be opened")
        exit()

    while True:
        distance = measure_distance()
        if distance is not None:
            logger.info(f"Distance: {distance} cm")

        if distance is not None and distance < OBSTACLE_THRESHOLD:
            stop()
            logger.info("Obstacle detected. Stopped!")
            time.sleep(0.5)
        else:
            ret, frame = cap.read()
            if not ret:
                logger.error("Failed to capture frame")
                break

            # Resize the frame to the desired dimensions
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            yellow_lane, mask = detect_yellow_lane(frame)
            edges = detect_lane_edges(mask)
            cropped_edges = region_of_interest(edges)
            lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, threshold=20, minLineLength=50, maxLineGap=150)

            steering_direction = compute_steering_direction(lines, frame.shape)

            # Draw detected lane lines on the frame
            line_image = draw_lines(frame, lines)
            combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
            cv2.imshow("Lane Detection", combo_image)

            if steering_direction == "forward":
                forward()
            elif steering_direction == "left":
                turn_left()
            elif steering_direction == "right":
                turn_right()
            else:
                stop()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    logger.info("Manual interrupt, stopping the car")
finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    logger.info("Cleaned up resources")
