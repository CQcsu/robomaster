# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import serial
import time

#  串口参数设置
port = "COM10"  # 端口
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

# 打开摄像头 设置摄像头分辨率640*480 13fps
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 摄像头相关参数设置
Lower = np.array([20, 70, 170])  # 设置颜色二值化 目前用黄色测试
Upper = np.array([40, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
detect_threshold = 0.04

while True:
    start = time.time()  # 处理程序开始时间
    cap_ret, frame = cap.read()

    src = cv2.flip(frame, 1)  # 视频显示左右翻转
    out = src.copy()
    screen_area = src.shape[0] * src.shape[1]

    img_Blur = cv2.GaussianBlur(src, (3, 3), 0)
    img_HSV = cv2.cvtColor(img_Blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_HSV, Lower, Upper)
    img_dilate = cv2.dilate(mask, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)
    img_canny = cv2.Canny(img_erode, 0, 250)
    img_canny_dilate = cv2.dilate(img_canny, kernel, iterations=1)

    max_cnt_area = -1
    max_cnt_index = -1
    max_cnt_x, max_cnt_y, max_cnt_w, max_cnt_h = -1, -1, -1, -1

    contours, hierarchy = cv2.findContours(img_canny_dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for i, cnt in enumerate(contours):
        # 轮廓信息提取
        area = cv2.contourArea(cnt)  # 获得轮廓面积
        x, y, w, h = cv2.boundingRect(cnt)
        if area > max_cnt_area:
            max_cnt_area = area
            max_cnt_index = i
            max_cnt_x, max_cnt_y, max_cnt_w, max_cnt_h = x, y, w, h

    if max_cnt_index != -1:
        cv2.drawContours(out, contours[max_cnt_index], -1, (255, 0, 0), 3)
    cv2.imshow("out", out)
    cv2.waitKey(10)
    cx, cy = max_cnt_y + max_cnt_h // 2, max_cnt_x + max_cnt_w // 2
    if max_cnt_area / screen_area > detect_threshold:
        if cy <= 320:
            distance = math.fabs(320 - cy)
            distance_int = int(distance)
            distance_info = distance_int & 0x00ff

            str_info = [0x55, 0x00, distance_info, 0xff]
            # print("左",str_info)

            num = ser.write(str_info)

        elif (cy > 320) and (cy <= 640):
            distance = math.fabs(320 - cy)
            distance_int = int(distance)
            distance_info = distance_int & 0x00ff

            str_info = [0x55, 0x01, distance_info, 0xff]
            # print("右",str_info)

            num = ser.write(str_info)
    end = time.time()  # 处理程序结束时间
    seconds = end - start
    print("time : {0} seconds".format(seconds))
    fps = 1 / seconds
    print("FPS:{}".format(fps))

# 关闭串口
Close_port(ser)
