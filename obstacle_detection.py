import RPi.GPIO as GPIO
import time

class ObstacleDetection:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.TRIG1 = 17
        self.ECHO1 = 27
        self.TRIG2 = 5
        self.ECHO2 = 6
        self.led = 22
        self.m11 = 16
        self.m12 = 12
        self.m21 = 21
        self.m22 = 20
        GPIO.setup(self.TRIG1, GPIO.OUT)
        GPIO.setup(self.ECHO1, GPIO.IN)
        GPIO.setup(self.TRIG2, GPIO.OUT)
        GPIO.setup(self.ECHO2, GPIO.IN)
        GPIO.setup(self.led, GPIO.OUT)
        GPIO.setup(self.m11, GPIO.OUT)
        GPIO.setup(self.m12, GPIO.OUT)
        GPIO.setup(self.m21, GPIO.OUT)
        GPIO.setup(self.m22, GPIO.OUT)
        GPIO.output(self.led, True)
        time.sleep(5)

    def measure_distance(self, trig_pin, echo_pin):
        GPIO.output(trig_pin, False)
        time.sleep(0.1)
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        pulse_start = time.time()
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
        pulse_end = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)

    def check_obstacle(self):
        distance1 = self.measure_distance(self.TRIG1, self.ECHO1)
        distance2 = self.measure_distance(self.TRIG2, self.ECHO2)
        avg_distance = (distance1 + distance2) / 2
        return avg_distance

    def stop(self):
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 0)

    def forward(self):
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 1)
        GPIO.output(self.m21, 1)
        GPIO.output(self.m22, 0)

    def backward(self):
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 1)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 1)

    def left(self):
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 1)
        GPIO.output(self.m22, 0)

    def right(self):
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 1)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 0)
