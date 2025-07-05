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
        GPIO.setwarnings(True)
        GPIO.setup(self.LASER_1, GPIO.OUT)
        GPIO.setup(self.LASER_2, GPIO.OUT)
        GPIO.setup(self.LED_RED, GPIO.OUT)
        GPIO.setup(self.LED_GREEN, GPIO.OUT)
        GPIO.setup(self.LED_BLUE, GPIO.OUT)
        GPIO.setup(self.DUOJI, GPIO.OUT)

        init_gpio(self)
        init_mg90s(self)


    def init_gpio(self):
        time.sleep(0.2)
        GPIO.output(self.LASER_1, GPIO.HIGH)
        GPIO.output(self.LASER_2, GPIO.HIGH)
        GPIO.output(self.LED_RED, GPIO.LOW)
        GPIO.output(self.LED_GREEN, GPIO.LOW)
        GPIO.output(self.LED_BLUE, GPIO.LOW)
        time.sleep(0.3)

    def init_mg90s(self):
        self.pwm_mg90s = GPIO.PWM(self.DUOJI, 50)
        self.pwm_mg90s.start(2.5)  # 转到0度
        time.sleep(0.5)

    def laser_set_1(self, set_time):
        GPIO.output(self.LASER_1, GPIO.LOW)
        time.sleep(set_time)
        GPIO.output(self.LASER_1, GPIO.HIGH)

    def laser_set_2(self, set_time):
        GPIO.output(self.LASER_2, GPIO.LOW)
        time.sleep(set_time)
        GPIO.output(self.LASER_2, GPIO.HIGH)

    def led_set_red(self, set_time):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LED_RED, GPIO.OUT)

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

    def duoji_set(self, angle_mode):
        #         self.pwm_sg90.start(2.5)  # 转到0度
        #         time.sleep(1.5)
        if angle_mode == 1:  # 90度
            self.pwm_mg90s.ChangeDutyCycle(7.5)
            time.sleep(0.5)
            # self.pwm_sg90.ChangeDutyCycle(7.5)
        if angle_mode == 2:  # 180度
            self.pwm_mg90s.ChangeDutyCycle(12)
            time.sleep(0.5)
            # self.pwm_sg90.ChangeDutyCycle(7.5)
        if angle_mode == 3:  # 转到0度
            self.pwm_mg90s.ChangeDutyCycle(2.5)
            time.sleep(0.5)
        pass

    def release(self):
        if not self.flag_gpio_has_released:
            GPIO.cleanup()
            self.oled.show_info(3, "gpio has released")
            self.flag_gpio_has_released = True

    def __del__(self):
        if not self.flag_gpio_has_released:
            GPIO.cleanup()
            self.oled.show_info(3, "gpio has released")
            self.flag_gpio_has_released = True
