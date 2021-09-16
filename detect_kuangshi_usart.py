# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import serial

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

# 打开摄像头 设置摄像头分辨率640*480
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 摄像头相关参数设置
Lower = np.array([20, 90, 170])  # 设置颜色二值化 目前用黄色测试
Upper = np.array([40, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

while True:
    cap_ret, frame = cap.read()
    src = cv2.flip(frame, 1)  # 视频显示左右翻转
    out = src.copy()

    img_Blur = cv2.GaussianBlur(src, (3, 3), 0)
    img_HSV = cv2.cvtColor(img_Blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_HSV, Lower, Upper)
    # cv2.imshow("mask", mask)
    img_dilate = cv2.dilate(mask, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)
    img_canny = cv2.Canny(img_erode, 0, 250)
    img_canny_dilate = cv2.dilate(img_canny, kernel, iterations=1)
    # cv2.imshow("img_canny", img_canny)
    # cv2.imshow("img_canny_dilate", img_canny_dilate)
    cv2.waitKey(10)
    max_cnt_area = -1
    max_cnt_index = -1
    max_cnt_x, max_cnt_y, max_cnt_w, max_cnt_h = -1, -1, -1, -1

    contours, hierarchy = cv2.findContours(img_canny_dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for i, cnt in enumerate(contours):
        # 轮廓信息提取
        area = cv2.contourArea(cnt)  # 获得轮廓面积
        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.drawContours(out, cnt, -1, (255, 0, 0), 3)
        if area > max_cnt_area:
            max_cnt_area = area
            max_cnt_index = i
            max_cnt_x, max_cnt_y, max_cnt_w, max_cnt_h = x, y, w, h

    if max_cnt_index != -1:
        cv2.drawContours(out, contours[max_cnt_index], -1, (255, 0, 0), 3)
    # cv2.imshow("out", out)
    cv2.waitKey(10)
    # print("当前屏幕shape", src.shape)
    # print(max_cnt_x, max_cnt_y, max_cnt_w, max_cnt_h)
    cx, cy = max_cnt_y + max_cnt_h // 2, max_cnt_x + max_cnt_w // 2
    
    if cy <= 320:
        # 第一个字符0代表矿石在屏幕左边，距离中间的像素distance用“ ”分开
        distance = math.fabs(320 - cy)
        print("矿石左边,距离中心点:", distance, "像素")
        str_distance = str(distance)
        left = '0'
        str_info = left + " " + str_distance
        num = ser.write(str_info.encode("gbk"))
        print("发送字节数num:", num)
    elif (cy > 320) and (cy <= 640):
        distance = math.fabs(320 - cy)
        print("矿石右边,距离中心点:", distance, "像素")
        str_distance = str(distance)
        right = '1'
        str_info = right + " " + str_distance
        num = ser.write(str_info.encode("gbk"))
        print("发送字节数num:", num)

# 关闭串口
Close_port(ser)

