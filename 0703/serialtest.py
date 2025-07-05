import serial
import time

# 串口参数，根据实际情况修改
SERIAL_PORT = "/dev/ttyUSB0"      # 串口设备名，比如COM3、/dev/ttyUSB0
BAUD_RATE = 9600              # 波特率
TIMEOUT = 1                     # 超时时间（秒）
LocationIndex = ["A1", "A2", "A3", "A4", "A5", "A6",
                 "B1", "B2", "B3", "B4", "B5", "B6"]
ItemIndex = [2, 3, 4, 5, 6, 7,
             10, 11, 12, 13, 14, 15]
def main():
    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print("打开串口 {SERIAL_PORT} 成功")
    except Exception as e:
        print("打开串口失败: {e}")
        return

    try:
        index = ser.readline()  # 示例：要调用的索引
        print("index=")
        print(index)
        # 要发送的数据，可以是字符串或字节流
        # 示例1：发送文本命令
        # data_str = "test\n"
        # ser.write(data_str.encode('utf-8'))   # 发送字符串需编码为bytes
        # print(f"已发送: {data_str.strip()}")
        #
        # time.sleep(0.1)  # 等待一会
        #
        # # 示例2：发送字节数据（如协议帧）
        # data_bytes = bytes([0x55, 0xAA, 0x40, 0x00, 0x00, 0x00, 0xAA, 0x00])
        # ser.write(data_bytes)
        # print(f"已发送字节数据: {data_bytes}")
        # for index in range(12):
        #     data_bytes = bytes(ItemIndex[index])
        #     ser.write(data_bytes)
            # data_str = LocationIndex[index]
            # ser.write(data_str.encode('utf-8'))   # 发送字符串需编码为bytes
            # print(f"已发送: {data_str.strip()}")

    except Exception as e:
        print("发送数据失败: {e}")
    finally:
        ser.close()
        print("串口已关闭")

if __name__ == "__main__":
    main()