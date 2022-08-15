#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sys
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
import time
import os
import re
import math
import dlib
from pyzbar import pyzbar
import numpy as np
import cv2
import threading
import img_pro
import datetime
import Serial_Servo_Running as SSR
import LeCmd
import getUsedSpace
import check_camera
import timeout_decorator
from PIL import Image, ImageDraw, ImageFont

print('''

----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Usage:
  -1 | --启动颜色识别玩法
  -2 | --启动人脸检测玩法
  -3 | --启动智能巡线玩法
  -4 | --启动手指个数识别玩法
----------------------------------------------------------
Example #1:
 显示图像,识别红绿蓝三种颜色
  python3 PC_function.py -1
----------------------------------------------------------

Version: --V2.2  2019/11/18
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可中断此次程序运行
----------------------------------------------------------
''')

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


# 数值映射
# 将一个数从一个范围映射到另一个范围
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#找出面积最大的轮廓
#参数为要比较的轮廓的列表
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None;

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max#返回最大的轮廓

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
##############################################

#################################################
# 颜色识别
def cv_color(frame):
    global width, height
    #颜色的字典
    color_range = {'red': [(0,43,46), (6, 255, 255)],
                  'blue': [(110,43,46), (124, 255,255)],
                  'green': [(35,43,46), (77, 255, 255)],
                  }
    range_rgb = {'red': (0, 0, 255),
                  'blue': (255, 0,0),
                  'green': (0, 255, 0),
                  }
    wd,hg = 320, 240
    dispose_frame = cv2.resize(frame, (wd, hg), interpolation = cv2.INTER_CUBIC) #将图片缩放     
    gs_frame = cv2.GaussianBlur(dispose_frame, (3,3), 0)#高斯模糊
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)#将图片转换到HSV空间

#################################################
# 智能巡线
def turn_left_right(angle):
    '''
    左转或者右转的动作组，根据实际需要转动的角度转动，  angle 为 负数 左转 ， 正数 右转
    :param angle: 转向的角度
    :return:
    '''

    pwm = int((-(angle * 4.167)/3) + 500)  # 1000(pwm 范围) / 240(舵机角度范围) = 4.166667； 500 中位位置
    pwm1 =int(((angle * 4.167)/3) + 500)
    if angle>0:
        #LeCmd.cmd_i001([100, 12, 1, pwm1, 2, 500, 3, pwm, 4, 570, 5, 500, 6, 430, 7, 500, 8, 500, 9, 480, 10, 480, 11, 490, 12, 500])
        #time.sleep(0.15)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 430, 7, 500, 8, 500, 9, 600, 10, 600, 11, 440, 12, 500])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 645, 10, 645, 11, 440, 12, 500])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 530, 10, 560, 11, 490, 12, 500])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 420, 7, 500, 8, 500, 9, 500, 10, 510, 11, 405, 12, 420])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 500, 10, 535, 11, 360, 12, 385])
        time.sleep(0.05)
        #LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 500, 10, 510, 11, 435, 12, 480])
        #time.sleep(0.05)
        #guaiwan
        LeCmd.cmd_i001([85, 12, 1, pwm1, 2, 500, 3, pwm, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 510, 10, 485, 11, 435, 12, 480])
        time.sleep(0.15)
    elif angle<0:
#test1        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 560, 5, 500, 6, 430, 7, 500, 8, 500, 9, 570, 10, 600, 11, 490, 12, 500])
#        time.sleep(0.05)
#        LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 570, 10, 600, 11, 490, 12, 500])
#        time.sleep(0.05)
#        LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 530, 10, 560, 11, 490, 12, 500])
#        time.sleep(0.05)
        #LeCmd.cmd_i001([100, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, pwm, 6, 420, 7, pwm1, 8, 500, 9, 500, 10, 510, 11, 515, 12, 515])
        #time.sleep(0.15)
#        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 420, 7, 500, 8, 500, 9, 500, 10, 510, 11, 405, 12, 420])
#        time.sleep(0.05)
#        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 500, 10, 510, 11, 405, 12, 420])
#        time.sleep(0.05)
#        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 500, 10, 510, 11, 435, 12, 480])
#        time.sleep(0.05)
#
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 430, 7, 500, 8, 500, 9, 600, 10, 600, 11, 440, 12, 500])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 645, 10, 645, 11, 440, 12, 500])
        time.sleep(0.05)
        #guaiwan
        LeCmd.cmd_i001([85, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, pwm, 6, 430, 7, pwm1, 8, 500, 9, 530, 10, 560, 11, 515, 12, 525])
        time.sleep(0.15)
        #LeCmd.cmd_i001([50, 12, 1, 500, 2, 430, 3, 500, 4, 500, 5, 500, 6, 430, 7, 500, 8, 500, 9, 530, 10, 560, 11, 490, 12, 500])
        #time.sleep(0.05)
        #test2 LeCmd.cmd_i001([100, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, pwm, 6, 420, 7, pwm1, 8, 500, 9, 500, 10, 510, 11, 515, 12, 515])
        #time.sleep(0.15)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 420, 7, 500, 8, 500, 9, 500, 10, 510, 11, 405, 12, 420])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, 500, 6, 500, 7, 500, 8, 570, 9, 500, 10, 535, 11, 360, 12, 385])
        time.sleep(0.05)
        LeCmd.cmd_i001([50, 12, 1, 500, 2, 500, 3, 500, 4, 570, 5, pwm, 6, 500, 7, pwm1, 8, 570, 9, 500, 10, 510, 11, 435, 12, 480])
        time.sleep(0.05)
        

# 机器人应该转的角度
line_deflection_angle = 0
line_cv_ok = False
Exit_thread = False
line_red_ok = False
center_x_pos = 240
red_times=0
def run_line():
    global line_cv_ok, center_x_pos, line_deflection_angle,line_red_ok
    while True:  
        if line_cv_ok:
            if 140 <= center_x_pos <= 340:
                SSR.running_action_group('demo15', 1)#test34
            else:
                turn_left_right(line_deflection_angle)
            line_cv_ok = False
        else:
            time.sleep(0.01)

line_th = threading.Thread(target=run_line)
line_th.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
line_th.start()

# 智能巡线
def line_patrol(f):
    global line_deflection_angle
    global line_cv_ok, center_x_pos,mask_red,red_times
    # 1.要识别的颜色字典
    color_dist = {'red': {'Lower': np.array([0, 50, 50]), 'Upper': np.array([6, 255, 255])},
                  'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
                  'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
                  'black': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 46])},
                  'white': {'Lower': np.array([0, 0, 221]), 'Upper': np.array([180, 30, 255])},
                  }

    roi = [ # [ROI, weight]
            (0,   120, 0, 480, 0.1), 
            (120, 240, 0, 480, 0.2), 
            (240, 360, 0, 480, 0.7)
           ]

    orgframe = cv2.GaussianBlur(f, (5, 5), 0)#高斯模糊，去噪
    hsv = cv2.cvtColor(orgframe, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])    
    # 腐蚀
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀
    mask = cv2.dilate(mask, None, iterations=2)
    centroid_x_sum = 0
    n = 0
    weight_sum = 0
    center_ = []
    
    for r in roi:
        n += 1
        blobs = mask[r[0]:r[1], r[2]:r[3]]        
        cnts = cv2.findContours(blobs , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓       
        if len(cnts):
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)#最小外接矩形
            if cv2.contourArea(c) >= 100:
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点           
                box[0, 1] = box[0, 1] + (n - 1)*120
                box[1, 1] = box[1, 1] + (n - 1)*120
                box[2, 1] = box[2, 1] + (n - 1)*120
                box[3, 1] = box[3, 1] + (n - 1)*120
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                cv2.drawContours(f, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形            
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点
                center_.append([center_x,center_y])            
                cv2.circle(f, (int(center_x), int(center_y)), 10, (0,0,255), -1)#画出中心点
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]
            
    if weight_sum is not 0:
        center_x_pos = centroid_x_sum / weight_sum
        
        #中间公式
        line_deflection_angle = 0.0
        line_deflection_angle = -math.atan((center_x_pos - 240)/(180))
        line_deflection_angle = line_deflection_angle*180.0/math.pi   
        
    line_cv_ok = True

##############################
    mask_red = cv2.inRange(hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
    if mask_red is not None:
        line_red_ok=True
        red_times+=1
        pass
    #if mask_red is not None and red_times>2000:
        #line_red_ok=False
        
        
#################################################
#########################       
operator = 1

if __name__ == '__main__':
    if len(sys.argv) > 1:#对传参长度进行判断
        mode = 0

        para = sys.argv[1]
        if para == "-1":        
            mode = 1
        elif para == "-2":
            mode = 2
        elif para == "-3":
            mode = 3
        else:
            print("异常：参数输入错误！")
            sys.exit()

        print('''--程序正常运行中......
              ''')
        
        th1 = threading.Thread(target = get_image)
        th1.setDaemon(True)
        th1.start()
        SSR.thread_runActing('huizhong', 1)
        while True:
            if ret and orgFrame is not None:
                try:
                    ret = False
                    t1 = cv2.getTickCount()
                    orgframe = cv2.resize(orgFrame, (480, 360), interpolation=cv2.INTER_LINEAR)  # 将图片缩放到
                    if mode == 3:
                        #cv_color(orgframe)
                        line_patrol(orgframe)
                    t2 = cv2.getTickCount()
                    time_r = (t2 - t1) / cv2.getTickFrequency()
                    fps = 1.0/time_r
                    cv2.putText(orgframe, "FPS:" + str(int(fps)),
                            (10, orgframe.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
                    cv2.imshow("orgframe", orgframe)
                    cv2.waitKey(1)
                except BaseException as e:
                    print(e)
                    continue
            else:
                time.sleep(0.01)
    else:
        print("异常：请重新运行，并输入参数！")
