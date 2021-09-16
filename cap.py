# -*- coding: utf-8 -*-
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置摄像头分辨率640*480
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cv2.namedWindow("out", cv2.WINDOW_NORMAL)
# cv2.namedWindow("out", cv2.WINDOW_FREERATIO)
cv2.setWindowProperty("out", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
# cap.set(cv2.CAP_PROP_FPS, 30)  # 设置摄像头帧率
# cap.set(cv2.CAP_PROP_EXPOSURE, 0)  # 设置摄像头曝光率 范围为0~4

while True:
    ret, frame = cap.read()
    cv2.imshow("out", frame)

    key = cv2.waitKey(10)
    if key == 27:
        break
