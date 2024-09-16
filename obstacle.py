import RPi.GPIO as GPIO
import time

# Setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pin Definitions
TRIG = 17
ECHO = 27
LED_PIN = 22

# Motor Pins
m11 = 16  # Motor 1 forward
m12 = 12  # Motor 1 backward
m21 = 21  # Motor 2 forward
m22 = 20  # Motor 2 backward

# Motor Speed (PWM Frequency)
SPEED = 80  # Change this value to control speed

# Pin Setup
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)

GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

# Setup PWM on motor control pins for speed control
motor1_forward = GPIO.PWM(m11, SPEED)
motor1_backward = GPIO.PWM(m12, SPEED)
motor2_forward = GPIO.PWM(m21, SPEED)
motor2_backward = GPIO.PWM(m22, SPEED)

# Start PWM with 0 duty cycle (off)
motor1_forward.start(0)
motor1_backward.start(0)
motor2_forward.start(0)
motor2_backward.start(0)

GPIO.output(LED_PIN, True)  # LED on
time.sleep(2)

# Function to measure distance
def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

# Motor Control Functions
def stop():
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(0)
    print('Stop')

def forward(speed=100):
    print('Moving Forward')
    motor1_forward.ChangeDutyCycle(speed)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(speed)
    motor2_backward.ChangeDutyCycle(0)

def backward(speed=100):
    print('Moving Backward')
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(speed)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(speed)

def turn_left(speed=100):
    print('Turning Left')
    motor1_forward.ChangeDutyCycle(0)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(speed)
    motor2_backward.ChangeDutyCycle(0)

def turn_right(speed=100):
    print('Turning Right')
    motor1_forward.ChangeDutyCycle(speed)
    motor1_backward.ChangeDutyCycle(0)
    motor2_forward.ChangeDutyCycle(0)
    motor2_backward.ChangeDutyCycle(0)

try:
    while True:
        distance = measure_distance()
        print(f"Distance: {distance} cm")

        if distance < 25:
            # Obstacle detected
            stop()
            print("Obstacle detected. Waiting for the road to clear...")

            # Wait until the road is clear
            while measure_distance() < 25:
                time.sleep(0.5)  # Check distance every 0.5 seconds

            print("Road is clear. Resuming movement.")

        # Clear path, move forward
        forward(speed=80)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    # Cleanup GPIO pins and stop PWM
    motor1_forward.stop()
    motor1_backward.stop()
    motor2_forward.stop()
    motor2_backward.stop()
    GPIO.cleanup()
