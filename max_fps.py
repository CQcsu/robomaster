# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import serial
import time

#  串口参数设置
port = "/dev/ttyS0"  # 端口
baudrate = 115200  # 波特率
timex = 5  # 超时时间设置：None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）


def Open_port(portx, bps, timeout):
    ret = False
    try:
        # 打开串口得到串口对象
        ser = serial.Serial(portx, bps, timeout=timeout)
        if ser.is_open:
            ret = True
            print("Open port")
    except Exception as e:
        print("---Error---", e)
    return ret, ser


def Close_port(serx):
    serx.close()
    print("Close port")


#  打开串口
ser_ret, ser = Open_port(port, baudrate, timex)

# 144p    (192×144，32帧/秒) 
# 240p    (320×240，26帧/秒) 
# 360p    (480×360，26帧/秒) 
# 480p    (640×480，13帧/秒)

# # 打开摄像头 设置摄像头分辨率640*480 13fps
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 打开摄像头 设置摄像头分辨率480*320 26fps
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 摄像头相关参数设置
Lower = np.array([20, 70, 170])  # 设置颜色二值化 目前用黄色测试
Upper = np.array([30, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
detect_threshold = 0.05

while True:
    
    start = time.time()  # 处理程序开始时间
    cap_ret, frame = cap.read()
    cv2.imshow("frame", frame)
    end = time.time()  # 处理程序结束时间
    seconds = end - start
    print("time : {0} seconds".format(seconds))
    fps = 1 / seconds
    print("FPS:{}".format(fps))

# 关闭串口
Close_port(ser)
