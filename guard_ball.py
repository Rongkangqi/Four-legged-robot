#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 小球追踪，x
import re
import cv2
import numpy as np
import time
import urllib
import threading
import signal
import pid
import LeCmd
import Serial_Servo_Running as SSR
import RPi.GPIO as GPIO
import check_camera
import timeout_decorator

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
key = 25
GPIO.setup(key, GPIO.IN, GPIO.PUD_UP)

orgFrame = None
ret = False
stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
width, height = 480, 360
font = cv2.FONT_HERSHEY_SIMPLEX

def get_cpu_mode():
    f_cpu_info = open("/proc/cpuinfo")
    for i in f_cpu_info:
        if re.search('Model', i):
            mode = re.findall('\d+', i)[0]
            break
    return mode

rpi = int(get_cpu_mode()) 

@timeout_decorator.timeout(0.5, use_signals=False)
def Camera_isOpened():
    global stream, cap
    cap = cv2.VideoCapture(stream)
    
try:
    Camera_isOpened()
    cap = cv2.VideoCapture(stream)
except:
    print('Unable to detect camera! \n')    
    check_camera.CheckCamera()
    
def get_image():
    global stream
    global orgFrame
    global ret
    global cap
    while True:
        try:
            if cap.isOpened():
                ret, orgFrame = cap.read()
            else:
                time.sleep(0.01)
        except:
            cap = cv2.VideoCapture(stream)
            print('Restart Camera Successful!') 

# 显示图像线程
th1 = threading.Thread(target=get_image)
th1.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()

# 要识别的颜色字典
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              }
# PS中的HSV范围，H是0-360，S是0-1，V（B）是0-1
# opencv中的HSV范围，H是0-180，S是0-255，V是0-255
# 青色颜色  48,112,216
# 把PS中H的值除以2，S乘255，V乘255，可以得到对应的opencv的HSV值
# cyan_rect = {'Lower': np.array([18, 72, 186]),
#              'Upper': np.array([78, 142, 246])}

run_one = False
SSR.running_action_group('0', 1)
while True:
    if orgFrame is not None:
        t1 = cv2.getTickCount()
        frame = orgFrame
        min_frame = cv2.resize(frame, (160, 120), interpolation=cv2.INTER_LINEAR)
        img_h, img_w = min_frame.shape[:2]
        img_center_x = img_w / 2
        img_center_y = img_h / 2
        # print(img_center_x, img_center_y)
        # 高斯模糊
        gs_frame = cv2.GaussianBlur(min_frame, (5, 5), 0)
        # 转换颜色空间
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
        # 查找颜色
        mask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
        # 腐蚀
        mask = cv2.erode(mask, None, iterations=2)
        # 膨胀
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)
        # 查找轮廓
        # cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            # 求出最小外接圆  原点坐标x, y  和半径
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius >= 25:  #
                cv2.circle(frame, (int(x * 3), int(y * 3)), 5, (0, 0, 255), -1)
                if run_one is False:
                    if x - img_center_x > 0:
                        SSR.running_action_group('guard_right', 1)
                        SSR.running_action_group('g_r', 1)
                    else:
                        SSR.running_action_group('guard_left', 1)
                        SSR.running_action_group('g_l', 1)
                    time.sleep(2)
                    SSR.running_action_group('kick_start', 1)
                    run_one = True
        else:
            run_one = False
        # 参数：图片, 起点, 终点, 颜色, 粗细
        # 画屏幕中心十字
        cv2.namedWindow("ball_track", cv2.WINDOW_AUTOSIZE)
        cv2.imshow('ball_track', frame)
        cv2.waitKey(1)
        t2 = cv2.getTickCount()
        time_r = (t2 - t1) / cv2.getTickFrequency() * 1000
        # print("%sms" % time_r)
    else:
        time.sleep(0.01)


