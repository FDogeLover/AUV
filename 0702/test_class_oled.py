import time
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306
import threading
import Adafruit_GPIO.SPI as SPI

class OledCtrl():
    def __init__(self):
        # Raspberry Pi pin configuration:
        self.RST = 24
        # Note the following are only used with SPI:
        self.DC = 23
        self.SPI_PORT = 0
        self.SPI_DEVICE = 0

        # 设置屏幕尺寸和I2C地址（具体参数根据你的设备进行调整）
        self.width = 128
        self.height = 64
        self.address = 0x3C

        # 分辨率选择一个即可
        # 128x32
        # disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
        # 128x64
        self.display = Adafruit_SSD1306.SSD1306_128_64(rst=self.RST)
        self.font = ImageFont.load_default()

    def init_oled(self):
        # 初始化库
        self.display.begin()
        # 获取屏幕大小
        self.width = self.display.width
        self.height = self.display.height
        # 清除屏幕内容
        self.display.clear()
        self.display.display()

        # 创建空白图像对象和绘制对象
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)

    def show_info(self, index_line, str_info, mode = 1):
        if mode == 1:
            # 清除上一帧的内容
            self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        if index_line == 1:
            self.draw.text((0, 0), str_info, font=self.font, fill=255)
        if index_line == 2:
            self.draw.text((0, 7), str_info, font=self.font, fill=255)
        if index_line == 3:
            self.draw.text((0, 15), str_info, font=self.font, fill=255)
        if index_line == 4:
            self.draw.text((0, 23), str_info, font=self.font, fill=255)
        if index_line == 5:
            self.draw.text((0, 31), str_info, font=self.font, fill=255)
        if index_line == 6:
            self.draw.text((0, 39), str_info, font=self.font, fill=255)
        if index_line == 7:
            self.draw.text((0, 47), str_info, font=self.font, fill=255)
        if index_line == 8:
            self.draw.text((0, 55), str_info, font=self.font, fill=255)
        # 将图像数据显示在SSD1306 OLED屏幕上
        self.display.image(self.image)
        self.display.display()

        # 暂停一秒
        time.sleep(0.5)

    def __del__(self):
        # 清除屏幕内容
        self.display.clear()
