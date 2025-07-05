import time
import RPi.GPIO as GPIO


class GpioCtrl:
    def __init__(self):
        self.KEY = 19
        self.DUOJI = 18
        self.LED_BLUE = 22
        self.LED_RED = 17
        self.LED_GREEN = 27
        self.LASER_1 = 4
        self.LASER_2 = 23

        self.flag_gpio_has_released = False

        GPIO.setmode(GPIO.BCM)
        #         GPIO.setwarnings(False)
        GPIO.setup(self.LASER_1, GPIO.OUT)
        GPIO.setup(self.LASER_2, GPIO.OUT)
        GPIO.setup(self.LED_RED, GPIO.OUT)
        GPIO.setup(self.LED_GREEN, GPIO.OUT)
        GPIO.setup(self.LED_BLUE, GPIO.OUT)
        GPIO.setup(self.DUOJI, GPIO.OUT)
        self.pwm = GPIO.PWM(self.DUOJI, 50)
        self.pwm.start(0)
        print("GPIO和PWM初始化成功")

    def duoji_move_servo(self, angle):
        print("舵机启动")
        #         self.pwm = GPIO.PWM(self.DUOJI, 50)
        #         self.pwm.start(0)

        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180) * 10;
        if duty_cycle > 12:
            duty_cycle = 12;
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)

    def laser_set_1(self, set_time):
        GPIO.output(self.LASER_1, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LASER_1, GPIO.LOW)

    def laser_set_2(self, set_time):
        GPIO.output(self.LASER_2, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LASER_2, GPIO.LOW)

    def led_set_red(self, set_time):
        GPIO.output(self.LED_RED, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_RED, GPIO.LOW)

    def led_set_green(self, set_time):
        GPIO.output(self.LED_GREEN, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_GREEN, GPIO.LOW)

    def led_set_blue(self, set_time):
        GPIO.output(self.LED_BLUE, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_BLUE, GPIO.LOW)

    def release(self):
        if not self.flag_gpio_has_released:
            GPIO.output(self.LED_BLUE, GPIO.HIGH)
            time.sleep(2)
            GPIO.output(self.LED_BLUE, GPIO.LOW)
            self.pwm.stop()  # 释放 PWM 资源
            GPIO.cleanup()
            print("GPIO已释放")
            self.flag_gpio_has_released = True

    def __del__(self):
        if not self.flag_gpio_has_released:
            self.pwm.stop()  # 释放 PWM 资源
            GPIO.cleanup()
            self.flag_gpio_has_released = True
            print("GPIO已释放")

