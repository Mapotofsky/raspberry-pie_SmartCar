# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
import collections
import sys
import numpy as np
import base64
import json
import os
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
from tencentcloud.common import credential
from tencentcloud.common.profile.client_profile import ClientProfile
from tencentcloud.common.profile.http_profile import HttpProfile
from tencentcloud.common.exception.tencent_cloud_sdk_exception import TencentCloudSDKException
from tencentcloud.iai.v20200303 import iai_client, models

# 小车电机引脚定义
IN1 = 20  # 前左电机是否开启
IN2 = 21  # 后左电机是否开启
IN3 = 19  # 前右电机是否开启
IN4 = 26  # 后右电机是否开启
ENA = 16  # 左侧电机控速
ENB = 13  # 右侧电机控速

# 小车按键定义
key = 8

# RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 蜂鸣器接口定义
alarm = 8

# 风扇引脚设置
fan = 2

# 循迹红外引脚定义
# TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口

# 超声波引脚定义
EchoPin = 0  # 回声脚
TrigPin = 1  # 触发脚

# 红外跟随模块引脚定义
FollowSensorLeft = 12
FollowSensorRight = 17

# 小车舵机定义
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOSTOP = 8

# 初始化上下左右角度为90度
ServoUpDownPos = 90

# 舵机引脚定义
ServoUpDownPin = 9

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


# 电机引脚初始化为输出模式
# 按键引脚初始化为输入模式
# 寻迹引脚初始化为输入模式
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_UpDownServo
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key, GPIO.IN)
    # 蜂鸣器
    GPIO.setup(alarm, GPIO.OUT, initial=GPIO.HIGH)
    # 风扇
    GPIO.setup(fan, GPIO.OUT, initial=GPIO.HIGH)
    # 循迹红外引脚
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    GPIO.setup(FollowSensorLeft, GPIO.IN)
    GPIO.setup(FollowSensorRight, GPIO.IN)
    # RGB三色灯设置为输出模式
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    # 舵机
    GPIO.setup(ServoUpDownPin, GPIO.OUT)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    # 设置舵机的频率和起始占空比
    pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
    pwm_UpDownServo.start(0)


# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    for i in range(1):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        # pwm_UpDownServo.ChangeDutyCycle(0)	#归零信号


# 摄像头舵机上下归位
def servo_updown_init():
    updownservo_appointed_detection(0)


# 舵机停止
def servo_stop():
    pwm_UpDownServo.ChangeDutyCycle(0)  # 归零信号


# 小车前进
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车后退
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车向左倒车
def bleft(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车向右倒车
def bright(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车左转
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车右转
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地左转
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地右转
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车停止
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


# 按键检测
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass


# 超声波函数
def Distance_test():
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    print("distance is %d " % (((t2 - t1) * 340 / 2) * 100))
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


# 巡线
def tracking(leftspeed, rightspeed):
    # 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    # 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

    # 四路循迹引脚电平状态
    # 0 0 0 0
    # 黑色横线，维持上次动作
    if TrackSensorLeftValue1 == False and TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and TrackSensorRightValue2 == False:
        run(leftspeed, rightspeed)

    # 四路循迹引脚电平状态
    # 0 0 X 0
    # 1 0 X 0
    # 0 1 X 0
    # 以上6种电平状态时小车原地右转
    # 处理右锐角和右直角的转动
    elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2
          == False) and TrackSensorRightValue2 == False:
        spin_right(40, 40)
        time.sleep(0.08)

    # 四路循迹引脚电平状态
    # 0 X 0 0
    # 0 X 0 1
    # 0 X 1 0
    # 处理左锐角和左直角的转动
    elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or
                                             TrackSensorRightValue2 == False):
        spin_left(40, 40)
        time.sleep(0.08)

    # 0 X X X
    # 最左边检测到
    elif TrackSensorLeftValue1 == False:
        spin_left(30, 30)

    # X X X 0
    # 最右边检测到
    elif TrackSensorRightValue2 == False:
        spin_right(30, 30)
    # 四路循迹引脚电平状态
    # X 0 1 X
    # 处理左小弯
    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
        left(0, 40)

    # 四路循迹引脚电平状态
    # X 1 0 X
    # 处理右小弯
    elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
        right(40, 0)
    # 四路循迹引脚电平状态
    # X 0 0 X
    # 处理直线
    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
        run(leftspeed, rightspeed)

    # 当为1 1 1 1时小车保持上一个小车运行状态


# 人脸识别
def takePhoto(picpath):
    """拍照
    """
    ret, img = cv2.VideoCapture(0).read()
    cv2.imwrite('/home/pi/Desktop/' + picpath, img)
    print('Photo is token.')


def result(res):
    """判断人脸识别结果
    """
    x = res['Score']
    threshold = 85

    if x > threshold:
        return True
    return False


def checkFace(picpath, personid):
    """人脸识别
    """
    # 先对拍摄到的图片进行base64编码
    with open(picpath, 'rb') as f:
        base64_data = base64.b64encode(f.read())
    s = base64_data.decode()
    pic_b64 = 'data:image/jpg;base64,' + s  # 将图片解码

    SecretId = "输入你的腾讯云id"  # id
    SecretKey = "输入你的腾讯云密钥"  # 密钥

    try:
        # 实例化一个认证对象
        cred = credential.Credential(
            SecretId, SecretKey)  # 入参需要传入腾讯云账户secretId，secretKey

        # 实例化一个http选项
        httpProfile = HttpProfile()
        httpProfile.endpoint = "iai.tencentcloudapi.com"  # 指定接入地域域名(默认就近接入)

        # 实例化一个client选项
        clientProfile = ClientProfile()
        clientProfile.httpProfile = httpProfile
        # 实例化要请求产品的client对象
        client = iai_client.IaiClient(cred, "ap-shanghai", clientProfile)

        # 实例化一个实例信息查询请求对象req
        req = models.VerifyPersonRequest()
        params = {"Image": pic_b64, "PersonId": personid, "QualityControl": 2}
        req.from_json_string(json.dumps(params))

        resp = client.VerifyFace(req)
        return True, json.loads(resp.to_json_string())

    except TencentCloudSDKException as err:
        print(err)
        return False, err


def faceDetection():
    """人脸检测
    """
    faceCascade = cv2.CascadeClassifier(
        '/home/pi/Desktop/haarcascade_frontalface_default.xml')

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # 宽
    cap.set(4, 480)  # 高

    count = 0
    wi = 0
    hi = 0
    while True:
        ret, img = cap.read()
        # img = cv2.flip(img, -1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(gray,
                                             scaleFactor=1.2,
                                             minNeighbors=5,
                                             minSize=(20, 20))

        for (x, y, w, h) in faces:
            # 框出人脸范围
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]
            wi = w
            hi = h

        cv2.imshow('video', img)

        k = cv2.waitKey(30) & 0xff
        if k == 27 or count == 30:  # 按'ESC'结束或等待时间截止
            break
        count += 1

    time.sleep(2)
    flag = False
    # 太小的不能算人脸
    if wi < 100 or hi < 100:
        flag = False
    else:
        flag = True
    # 摄像头调用完毕
    cap.release()
    cv2.destroyAllWindows()
    return flag


def checking(picpath='tmp.jpg', personid='1'):
    """识别总调用
    """
    i = 0
    flag = 0
    # 摄像头纵向舵机抬至45度，方便人脸识别
    updownservo_appointed_detection(45)
    time.sleep(1)
    servo_stop()
    # 检测是否有人脸
    havePer = faceDetection()
    if not havePer:
        # 没有人脸，返回-1
        servo_updown_init()
        return -1

    # 识别失败则再次识别，最多重复3次
    while i < 3:
        takePhoto(picpath)
        # 调用腾讯云API得到人脸验证结果
        rflag, info = checkFace(picpath, personid)
        if rflag:
            # 根据验证分数得到结果
            if result(info):
                # 验证通过，闪绿灯
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.HIGH)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.HIGH)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                print('好耶')
                flag = 1
                # 验证通过，返回1
                servo_updown_init()
                return flag
            else:
                # 验证未通过，闪红灯
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(0.05)
                print('你谁啊')

        # 验证不通过，返回0
        i += 1

    servo_updown_init()
    return flag


# 闪灯
def ColorLED():
    # 循环显示7种不同的颜色
    i = 10
    while i > 0:
        GPIO.output(LED_R, GPIO.HIGH)  # 红
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.LOW)  # 绿
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.LOW)  # 蓝
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.HIGH)  # 红绿
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.HIGH)  # 红蓝
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.LOW)  # 绿蓝
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(LED_R, GPIO.LOW)  # 灭
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(0.05)
        i -= 1


# 蜂鸣器报警
def alarming():
    GPIO.setup(8, GPIO.OUT)  # 将BCM12号引脚设置为输出模式
    pwm = GPIO.PWM(8, 1000)  # 设置BCM12号引脚, 设置pwm频率为1000HZ
    pwm.start(50)  # 设置初始占空比（范围：0.0 <= dc <= 100.0)
    # pwm.ChangeFrequency(freq)   # freq 为设置的新频率，单位为 Hz
    # pwm.ChangeDutyCycle(dc)     # dc 为设置的新的占空比 范围：0.0-100.0
    i = 1000  # 频率
    dirs = 1  # 频率递增方向，1为正，-1为负
    try:
        count5 = 0
        while count5 < 300:  # 蜂鸣持续1.5秒
            pwm.ChangeFrequency(i)  # 为设置的新频率，单位为 Hz
            count5 += 1
            i = i + 100 * dirs  # 当前频率加上要增加的频率(10)乘以方向
            time.sleep(0.05)  # 延时0.05秒
            print('pwm当前频率为: %d ' % i)  # 控制台打印当前的频率
            if i >= 2000:  # 如果当前频率大于2000hz，方向改为负
                dirs = -1
            elif i <= 1000:  # 如果当前频率小于1000hz，方向改为正
                dirs = 1
        GPIO.setup(alarm, GPIO.OUT, initial=GPIO.HIGH)  # 初始化停止蜂鸣
    finally:
        pass


# 对主人启动风扇
def fanBlow():
    GPIO.output(fan, not GPIO.input(fan))  # 风扇持续4秒
    time.sleep(4)
    GPIO.setup(fan, GPIO.OUT, initial=GPIO.HIGH)


# 播放音频
def sound():
    path = "/home/pi/Desktop/test.mp3"
    os.system('mplayer %s' % path)


# 发送邮件
def sendEmail():
    # 设置服务器所需信息
    fromaddr = '输入你的邮箱'
    password = '输入你的qq邮箱授权码'  # 邮箱授权码
    toaddrs = ['输入你要发送的邮箱', '输入你要发送的邮箱']

    # 设置email信息
    content = 'hello, this is email content.'
    textApart = MIMEText(content)

    imageFile = '/home/pi/Desktop/tmp.jpg'
    imageApart = MIMEImage(
        open(imageFile, 'rb').read(),
        imageFile.split('.')[-1])
    imageApart.add_header('Content-Disposition',
                          'attachment',
                          filename=imageFile)

    m = MIMEMultipart()
    m.attach(textApart)
    m.attach(imageApart)
    m['Subject'] = 'title'

    # 登录并发送邮件
    try:
        server = smtplib.SMTP('smtp.qq.com')  # qq邮箱服务器地址
        server.login(fromaddr, password)
        server.sendmail(fromaddr, toaddrs, m.as_string())
        print('success')
        server.quit()

    except smtplib.SMTPException as e:
        print('error', e)  # 打印错误


# 红外跟随
def infrared_follow():
    # 遇到跟随物,红外跟随模块的指示灯亮,端口电平为LOW
    # 未遇到跟随物,红外跟随模块的指示灯灭,端口电平为HIGH
    LeftSensorValue = GPIO.input(FollowSensorLeft)
    RightSensorValue = GPIO.input(FollowSensorRight)

    if LeftSensorValue == False and RightSensorValue == False:
        run(13, 13)  # 当两侧均检测到跟随物时调用前进函数
    elif LeftSensorValue == False and RightSensorValue == True:
        spin_left(60, 60)  # 左边探测到有跟随物，有信号返回，原地向左转
        time.sleep(0.002)
    elif RightSensorValue == False and LeftSensorValue == True:
        spin_right(60, 60)  # 右边探测到有跟随物，有信号返回，原地向右转
        time.sleep(0.002)
    elif RightSensorValue == True and LeftSensorValue == True:
        brake()  # 当两侧均未检测到跟随物时停止
        return False  # 返回是否有跟随物
    return True


# 倒车入库
def back_into_garage(leftspeed, rightspeed):
    print("daocheing")
    # 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
    # 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
    TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

    # 四路循迹引脚电平状态
    # 0 0 0 0
    # 遇到黑色横线，倒车入库
    if TrackSensorLeftValue1 == False and TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and TrackSensorRightValue2 == False:
        back(10, 10)
        time.sleep(1.5)
        spin_left(80, 80)
        time.sleep(1.02)
        back(10, 10)
        time.sleep(1.45)
        sys.exit(0)

    # 四路循迹引脚电平状态
    # 0 0 X 0
    # 1 0 X 0
    # 0 1 X 0
    # 以上6种电平状态时小车原地右转
    # 处理右锐角和右直角的转动
    elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2
          == False) and TrackSensorRightValue2 == False:
        spin_right(40, 40)
        time.sleep(0.08)

    # 四路循迹引脚电平状态
    # 0 X 0 0
    # 0 X 0 1
    # 0 X 1 0
    # 处理左锐角和左直角的转动
    elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or
                                             TrackSensorRightValue2 == False):
        spin_left(40, 40)
        time.sleep(0.08)

    # 0 X X X
    # 最左边检测到
    elif TrackSensorLeftValue1 == False:
        spin_left(30, 30)

    # X X X 0
    # 最右边检测到
    elif TrackSensorRightValue2 == False:
        spin_right(30, 30)

    # 四路循迹引脚电平状态
    # X 0 1 X
    # 处理左小弯
    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
        left(0, 40)

    # 四路循迹引脚电平状态
    # X 1 0 X
    # 处理右小弯
    elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
        right(40, 0)

    # 四路循迹引脚电平状态
    # X 0 0 X
    # 处理直线
    elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
        run(leftspeed, rightspeed)

    # 当为1 1 1 1时小车保持上一个小车运行状态


# 颜色识别组合三种操作
# 颜色库
def getColorList():
    dict = collections.defaultdict(list)

    # red
    lower_red = np.array([0, 43, 46])  # 红色色域低值
    upper_red = np.array([10, 255, 255])  # 红色色域高值
    color_list_red = []
    color_list_red.append(lower_red)
    color_list_red.append(upper_red)
    dict['red'] = color_list_red

    # green
    lower_green = np.array([35, 43, 46])  # 绿色色域低值
    upper_green = np.array([77, 255, 255])  # 绿色色域高值
    color_list_green = []
    color_list_green.append(lower_green)
    color_list_green.append(upper_green)
    dict['green'] = color_list_green

    # blue
    lower_blue = np.array([100, 43, 46])  # 蓝色色域低值
    upper_blue = np.array([124, 255, 255])  # 蓝色色域高值
    color_list_blue = []
    color_list_blue.append(lower_blue)
    color_list_blue.append(upper_blue)
    dict['blue'] = color_list_blue

    return dict


def get_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 转化为HSV颜色空间
    maxsum = 0
    color = None
    color_dict = getColorList()  # 获取颜色库
    for d in color_dict:
        # 二值化功能，主要是将在两个阈值内的像素值设置为白色（255），而不在阈值区间内的像素值设置为黑色（0）
        mask = cv2.inRange(hsv, color_dict[d][0], color_dict[d][1])
        mask = cv2.erode(mask, None, iterations=2)  # 腐蚀函数
        mask = cv2.GaussianBlur(mask, (3, 3), 0)  # 对图像进行高斯滤波，去除噪声
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]  # 寻找轮廓
        sum = 0
        for c in cnts:
            sum += cv2.contourArea(c)  # 计算图像轮廓的面积
        if sum > maxsum:  # 选出色域面积最大的颜色即为当前所识别出的颜色
            maxsum = sum
            color = d

    return color


# 红绿蓝卡牌颜色命令
def videox_color():
    vix = cv2.VideoCapture(0)  # 调用摄像头
    tu, frame = vix.read()
    frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 图像进行高斯滤波，去除噪声
    for i in range(30):
        cv2.imshow("video", frame)  # 显示调取画面
        cv2.waitKey(30)
    color = get_color(frame)
    # 摄像头调用结束
    vix.release()
    cv2.destroyAllWindows()
    print(color)
    return color  # 返回识别出的颜色


def colorcontrol():
    color = videox_color()  # 识别颜色
    if color == "blue":  # 识别出蓝色，点亮彩灯，2秒后播放音频
        ColorLED()
        time.sleep(2)
        sound()
    elif color == "green":  # 识别出绿色，进行红外跟随
        while infrared_follow():
            pass
        print("end follow")
        time.sleep(3)
    elif color == "red":  # 识别出红色，进行倒车入库
        print("daoche")
        while True:
            back_into_garage(20, 20)


# 小车行驶中遇到障碍物进行绕开操作
def moveAway():
    spin_left(18, 18)  # 原地左转 0.7s
    time.sleep(0.7)
    brake()
    run(10, 10)  # 前进 1s
    time.sleep(1)
    spin_right(15, 15)  # 原地右转 0.8s
    time.sleep(0.8)
    run(10, 10)  # 前进 1.9s
    time.sleep(1.9)
    spin_right(20, 20)  # 原地右转 0.5s
    time.sleep(0.5)
    run(10, 10)  # 前进1.2s
    time.sleep(1.2)
    spin_left(20, 20)  # 原地左转0.8s
    time.sleep(0.8)


# 延时2s
time.sleep(2)

try:
    init()  # 初始化
    key_scan()  # 按键检测
    while True:
        distance = Distance_test()  # 超声波实时测距
        if distance > 50:
            tracking(20, 20)  # 当距离障碍物较远时高速巡线前进
        elif 20 <= distance <= 50:
            tracking(10, 10)  # 当快靠近障碍物时低速巡线前进
        elif distance < 20:  # 当靠近障碍物
            brake()  # 停车2秒
            time.sleep(2)
            recognition = checking()  # 进行人脸识别
            if recognition == 1:  # 识别到主人，主人可用红绿蓝三色卡牌控制小狗
                fanBlow()  # 给主人吹风
                time.sleep(2)
                count = 0
                print("举起卡牌")
                colorcontrol()  # 进行颜色识别
            elif recognition == 0:  # 识别到陌生人
                sendEmail()  # 发送邮件至陌生人照片主人邮箱
                time.sleep(1)
                alarming()  # 发送完邮件1秒后蜂鸣报警
            elif recognition == -1:  # 未识别到人，判断为障碍物
                moveAway()  # 执行避障操作

except KeyboardInterrupt:
    pass
pwm_ENA.stop()  # 左电机停止运行
pwm_ENB.stop()  # 右电机停止运行
GPIO.cleanup()  # 管脚清零
