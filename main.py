from multiprocessing import Manager, Process, Pool, Lock, Condition
import time
import cv2
import numpy as np
import math
import serial



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


# 摄像头相关参数设置
Lower = np.array([23, 80, 150])  # 设置颜色二值化 目前用黄色测试
Upper = np.array([35, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
detect_threshold = 0.03


producer_mutex = Condition()  # 锁，控制队列访问避免脏数据
consumer_mutex = Condition()  # 锁，控制队列访问避免脏数据
queue_num = 7
thread_producer_num = 6
thread_consumer_num = 1
# 因为串口只允许一个进程打开 所以只定义一个消费者进程，专门用来发送串口消息


def process_image(frame):
    src = cv2.flip(frame, 1)  # 视频显示左右翻转
    out = src.copy()
    screen_area = src.shape[0] * src.shape[1]

    img_Blur = cv2.GaussianBlur(src, (3, 3), 0)
    img_HSV = cv2.cvtColor(img_Blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_HSV, Lower, Upper)
    # cv2.imshow("mask", mask)
    # cv2.waitKey(10)
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
    cv2.imshow("canny", out)
    cv2.waitKey(10)

    cx, cy = max_cnt_y + max_cnt_h // 2, max_cnt_x + max_cnt_w // 2
    if max_cnt_area / screen_area > detect_threshold:
        if cy <= 320:
            distance = math.fabs(320 - cy)
            distance_int = int(distance)
            distance_info = distance_int & 0x00ff

            str_info = [0x55, 0x00, distance_info, 0xff]
            return str_info

        elif (cy > 320) and (cy <= 640):
            distance = math.fabs(320 - cy)
            distance_int = int(distance)
            distance_info = distance_int & 0x00ff

            str_info = [0x55, 0x01, distance_info, 0xff]
            return str_info


def producer_task(work_queue):
    # 打开摄像头 设置摄像头分辨率640*480
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        cap_ret, frame = cap.read()
        cv2.imshow("out", frame)  # ######sc
        cv2.waitKey(10)
        # 成功读取图片将图片存入仓库
        if cap_ret is False:
            print("图片读取失败!!!")
            break
        else:  # 如果图片仓库满了 直接舍弃 没满将图片存入仓库
            work_queue.put(frame)
            # print("put--------frame----")


def producer(name,  work_queue, strinfo_queue):
    while True:
        # print(work_queue.empty())
        if not work_queue.empty():  # 如果图片仓库不为空获得一张图片
            producer_mutex.acquire()  # 处理数据前先上锁
            frame = work_queue.get()  # 获得一张图片
            producer_mutex.release()  # 没有图片时,解锁并退出本次循环
            # cv2.imshow("frame", frame)  # ######sc
            # cv2.waitKey(10)
        else:
            continue
        str_info = process_image(frame)
        # print("处理图片得到信息：" + str(str_info))  # ######sc
        if strinfo_queue.full():  # 如果信息仓库满直接放弃本次信息， 没满将信息加入信息仓库
            pass  # 按照ljj说法这里可能会发0x0000消息
        elif str_info is None:
            # 如果没有识别到矿石 发送0x00消息
            str_info = [0x00, 0x00, 0x00, 0x00]
            strinfo_queue.put(str_info)
        else:
            strinfo_queue.put(str_info)
            # mutex.notify_all()  # 通知所有等待消费者，生产完成可以消费
            # print("------------")


def consumer(name, strinfo_queue):
    #  串口参数设置
    port = "/dev/ttyS0"  # 端口
    # port = "COM10"  # 端口
    baudrate = 115200  # 波特率
    timex = 5  # 超时时间设置：None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
    #  打开串口
    ser_ret, ser = Open_port(port, baudrate, timex)
    while True:
        consumer_mutex.acquire()  # 处理数据前先上锁
        if strinfo_queue.empty():
            # mutex.wait()  # 仓库为空进入等待阻塞进程   这里可能有问题 队列为空或许应该直接下次循环获得下次信息不应该等待notify
            consumer_mutex.release()
            continue
        str_info = strinfo_queue.get()
        consumer_mutex.release()
        num = ser.write(str_info)  # 发送串口信息
        print("发送信息: ", str_info)

    # 关闭串口， 循环异常
    Close_port(ser)


if __name__ == '__main__':
    pool = Pool(processes=8)
    manager = Manager()
    work_queue = manager.Queue(queue_num)
    strinfo_queue = manager.Queue(queue_num)

    pool.apply_async(producer_task, args=(work_queue,))
    for i in range(thread_producer_num):
        pool.apply_async(producer, args=(i, work_queue, strinfo_queue, ))
    for i in range(thread_consumer_num):
        pool.apply_async(consumer, args=(i, strinfo_queue,))

    while True:
        continue

