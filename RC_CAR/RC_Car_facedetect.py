# -*- coding: utf-8 -*-

import pygame
import numpy as np
from pygame.locals import *
import sys
import RPi.GPIO as GPIO
import time

import cv2

cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


pygame.init()
screen = pygame.display.set_mode((640, 480))

pygame.display.set_caption('RC_Car_Control')
pygame.mouse.set_visible(0)

# 모터 상태
STOP = 0
FORWARD = 1
BACKWORD = 2

# 모터 채널
HANDLE = 0
ENGINE = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

# 핀 설정 함수
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴 
    pwm = GPIO.PWM(EN, 100) 
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm

# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):
    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    #뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

        
# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    if ch == ENGINE:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)
def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()

def draw_str(dst, x, y, s):
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0),
                                                    thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255),
                                                    thickness = 2, lineType=cv2.LINE_AA)
def detect(img, cascade):
             rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=3,
                                minSize=(20,20), flags = cv2.CASCADE_SCALE_IMAGE)
             if len(rects) == 0:
                 return []
             rects[:,2:] += rects[:,:2]
             return rects

def draw_rects(img, rects, color):
             for x1, y1, x2, y2 in rects:
                 cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

# GPIO 모드 설정 
GPIO.setmode(GPIO.BCM)
      
#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴 
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

speed = 100
max_speed = 100

up = False
down = False
left = False
right = False

if __name__ == '__main__':
    cascade_fn = "haarcascade_frontalface_alt.xml"
    nested_fn = "haarcascade_eye.xml"
    cascade = cv2.CascadeClassifier(cascade_fn)
    nested = cv2.CascadeClassifier(nested_fn)
  
    #제어 시작
    while True:
        ret, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        t = clock()
        rects = detect(gray, cascade)
        vis = img.copy()
        draw_rects(vis, rects, (0, 255, 0))
        for x1, y1, x2, y2 in rects:
                roi = gray[y1:y2, x1:x2]
                vis_roi = vis[y1:y2, x1:x2]
                subrects = detect(roi.copy(), nested)
                draw_rects(vis_roi, subrects, (255, 0, 0))
        dt = clock() -t
                 
        draw_str(vis, 20, 20, 'time: %.1f ms'% (dt*1000))
        cv2.imshow('facedetect', vis)
        key = cv2.waitKey(10)
        if key == 27:
            break
        if key == ord(' '):
            cv2.imwrite('capture1.jpg',img)
            
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == K_UP:
                    print("KEYDOWN K_UP")
                    up = True
                elif event.key == K_DOWN:
                    print("KEYDOWN K_DOWN")
                    down = True
                elif event.key == K_LEFT:
                    print("KEYDOWN K_LEFT")
                    left = True
                elif event.key == K_RIGHT:
                    print("KEYDOWN K_RIGHT")
                    right = True
            elif event.type == KEYUP:
                if event.key == K_UP:
                    print("KEYUP K_UP")
                    up = False
                elif event.key == K_DOWN:
                    print("KEYUP K_DOWN")
                    down = False
                elif event.key == K_LEFT:
                    print("KEYUP K_LEFT")
                    left = False
                elif event.key == K_RIGHT:
                    print("KEYUP K_RIGHT")
                    right = False
        if up == True and down == False:
            setMotor(ENGINE, speed, FORWARD)
        elif up == False and down == True:
            setMotor(ENGINE, speed, BACKWORD)
        else:
            setMotor(ENGINE, max_speed, STOP)
        if left == True and right == False:
            setMotor(HANDLE, max_speed, FORWARD)
        elif left == False and right == True:
            setMotor(HANDLE, max_speed, BACKWORD)
        else:
            setMotor(HANDLE, max_speed, STOP)
    # 종료
    cv2.destoryAllWindows()
    GPIO.cleanup()
