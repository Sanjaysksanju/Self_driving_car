#### SELF DRIVING CAR USING RASPBERRY PI

---

## Overview

This project demonstrates a lane-detecting robot car with obstacle avoidance, built using a Raspberry Pi, camera, ultrasonic sensor, and motor control. The robot detects lane markings using computer vision, navigates through them, and stops or adjusts its path when it encounters obstacles.

---

## System Architecture

### Hardware Components:

1. **Raspberry Pi 4/3 B+**
2. **Ultrasonic Sensor** (HC-SR04) for obstacle detection
3. **Motor Driver** (L293D) to control two DC motors
4. **Camera Module** (Pi Camera or USB Camera) for lane detection
5. **DC Motors** for driving the robot
6. **Jumper Wires**, **Breadboard**, and **Power Supply** for the Raspberry Pi
7. **LED** for signaling system status (optional)

### Wiring Diagram:

- **Ultrasonic Sensor (HC-SR04)**:
  - VCC: 5V
  - GND: Ground
  - TRIG: GPIO 17
  - ECHO: GPIO 27
- **DC Motors** (connected to motor driver):
  - Motor1 Forward: GPIO 16
  - Motor1 Backward: GPIO 12
  - Motor2 Forward: GPIO 21
  - Motor2 Backward: GPIO 20
- **LED**:
  - GPIO 22 for LED status

A simple wiring diagram would show the connections between the Raspberry Pi, motors, ultrasonic sensor, and camera module. You can create this using **Fritzing** for detailed illustrations.

---

## Software Overview

This robot uses the following primary components:

### 1. **Lane Detection using OpenCV:**

- The lane detection algorithm converts a camera frame to the HSV color space, detects the yellow lane lines, applies edge detection (Canny), and extracts lane boundaries using the Hough Transform.
- The algorithm computes the direction based on the detected lanes and steers the robot accordingly (left, right, forward).

### 2. **Obstacle Detection using Ultrasonic Sensor:**

- The ultrasonic sensor continuously measures the distance from obstacles.
- If an obstacle is detected within the defined threshold distance (`OBSTACLE_THRESHOLD`), the robot stops until the obstacle is cleared.

### 3. **Motor Control:**

- The robot is controlled via PWM signals sent to the GPIO pins connected to the motor driver. The functions `forward()`, `backward()`, `turn_left()`, and `turn_right()` control the direction and speed of the motors.

---

## Code Breakdown

### Key Functions:

- **Lane Detection**:
  - `detect_yellow_lane(frame)`: Identifies yellow lanes in the captured camera frame.
  - `detect_lane_edges(mask)`: Detects edges of the lane using the Canny Edge Detector.
  - `compute_steering_direction(lines, frame_shape)`: Determines steering direction based on detected lane positions.

- **Motor Control**:
  - `forward()`, `backward()`, `turn_left()`, and `turn_right()`: These functions control the motors using PWM signals for the speed and direction of the robot.

- **Obstacle Detection**:
  - `measure_distance()`: Uses the ultrasonic sensor to measure distance to nearby obstacles.
  - If an obstacle is detected within the threshold, the robot halts.

### Flowchart of the Main Loop:

1. Capture frame from the camera.
2. Detect yellow lanes and compute the steering direction.
3. Measure distance from any obstacles using the ultrasonic sensor.
4. Move forward, turn left, or turn right based on lane detection and obstacle data.
5. Stop when an obstacle is detected.

---

## Installation

### Prerequisites:

1. **Python 3.x** installed on Raspberry Pi
2. **OpenCV** for Python: Install using the following commands:
    ```bash
    sudo apt update
    sudo apt install python3-opencv
    ```
3. **RPi.GPIO**: A Python library to control GPIO pins on the Raspberry Pi.
    ```bash
    sudo apt install python3-rpi.gpio
    ```

### Setup:

1. Clone the repository or copy the script into your Raspberry Pi:
    ```bash
    git clone https://github.com/your-repo/raspberry-lane-detection.git
    cd raspberry-lane-detection
    ```

2. Install required Python libraries:
    ```bash
    pip3 install numpy
    pip3 install opencv-python
    pip3 install RPi.GPIO
    ```

3. Ensure that the camera module is enabled:
    ```bash
    sudo raspi-config
    ```
   Under **Interfacing Options**, enable the camera.

4. Connect the hardware components according to the wiring diagram.

5. Run the program:
    ```bash
    python3 lane_detection_robot.py
    ```

---

## Running the Robot

Once the script is running, the robot will:

1. Continuously capture frames using the camera.
2. Detect yellow lanes and steer accordingly.
3. Stop if any obstacles are detected within a certain range.
4. Automatically resume lane-following once the obstacle is cleared.

Press `q` to quit the application.

---

## Video & Image Examples

### 1. **Lane Detection:**
(![image](https://github.com/user-attachments/assets/efba4134-98c1-4aa0-8186-99f75f6f10de)

*Explanation*: The robot captures the road ahead and identifies yellow lane lines, as shown in the green overlay.

---

### 2. **Obstacle Detection:**
(![image](https://github.com/user-attachments/assets/b8231d5e-1e38-4428-9c5f-2b618ca786e1)

*Explanation*: When an obstacle is detected within the threshold distance (25 cm), the robot stops, as indicated by the log messages.

---

## Troubleshooting

### Common Issues:
1. **Camera not working**:
   - Ensure the camera is correctly connected and enabled in `raspi-config`.
   - Use `raspistill -v -o test.jpg` to test if the camera is working.

2. **Distance measurement errors**:
   - Verify wiring for the ultrasonic sensor.
   - Make sure no objects interfere with the sensor readings.

3. **Lane detection inaccuracies**:
   - Check the lighting conditions, as poor lighting can affect the yellow color detection.
   - Adjust the HSV range for yellow in `detect_yellow_lane()` if the lane color is different in your environment.

---

## Future Improvements

1. **Dynamic Lane Detection**: Improve lane detection robustness by adding support for detecting other lane colors or shapes.
2. **Speed Adjustment**: Vary the robot's speed based on the curvature of the lane or proximity to obstacles.
3. **Wireless Control**: Implement remote control or monitoring via a web interface.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

## Contributors

- [Sanjay S K](aksha9538@gmail.com)
- [Supriya G K](gksupriya09@gmail.com)
- [Pranathi S](pranathisns25@gmail.com)

---
