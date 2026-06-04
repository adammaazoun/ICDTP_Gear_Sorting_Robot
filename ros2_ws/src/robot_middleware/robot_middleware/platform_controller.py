import time
import RPi.GPIO as GPIO


class PlatformController:

    SERVO_PIN = 18

    def __init__(self):

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(
            self.SERVO_PIN,
            50
        )

        self.pwm.start(0)


    def rotate_180(self):

        print("Rotating platform")

        self.set_angle(180)

        time.sleep(1)

        self.set_angle(0)

        time.sleep(1)


    def set_angle(self, angle):

        duty = 2 + angle / 18

        self.pwm.ChangeDutyCycle(duty)

        time.sleep(0.5)

        self.pwm.ChangeDutyCycle(0)


    def cleanup(self):

        self.pwm.stop()

        GPIO.cleanup()