import vrep
import numpy as np
import cv2
import time
import math
import threading

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP

color_range = {
'red': [(85, 164, 0), (255, 255, 255)],
'green': [(157, 0, 0), (252, 110, 255)],
'blue': [(108, 0, 0), (255, 255, 106)],
'black': [(0, 0, 0), (62, 255, 250)],
'white': [(217, 0, 0), (255, 255, 250)],
              }

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

#检测并返回最大面积
def getAreaMaxContour(contours, area=1):
        contour_area_max = 0
        area_max_contour = None

        for c in contours :
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > area:#面积大于1
                    area_max_contour = c
        return area_max_contour

get_line = False
center_x = 0
speed = 5
r_speed, l_speed = 0, 0
x_pid = PID(P=0.2, I=0, D=0)
def Tracing(orgimage):
    global get_line, center_x, speed
    global l_speed, r_speed
    orgframe_gas = cv2.GaussianBlur(orgimage, (3, 3), 0)  # 高斯模糊，去噪
    orgframe_lab = cv2.cvtColor(orgframe_gas, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    orgframe_mask = cv2.inRange(orgframe_lab, color_range['black'][0],
                                color_range['black'][1])  # 根据hsv值对图片进行二值化
    opened = cv2.morphologyEx(orgframe_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
    # cv2.imshow('closed', closed)
    center_x = 0
    blobs = closed
    cnts = cv2.findContours(blobs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
    cnt_large = getAreaMaxContour(cnts)  # 找到最大面积的轮廓
    if cnt_large is not None:
        rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        pt1_x, pt1_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)  # 画出四个点组成的矩形
        center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # 中心点
        cv2.circle(img, (int(center_x), int(center_y)), 10, (0, 0, 255), -1)  # 画出中心点
        x_pid.SetPoint = orgimage.shape[-2]/2  # 将图像中心作为设定的目标值
        # pid计算
        x_pid.update(center_x)  # 将当前获取的跑道中心值作为输入值
        # pid输出
        out = int(x_pid.output)

        # 对输出做限制，不能超过速度范围
        if speed - out < -10:
            out = 10 + speed
        elif speed - out > 10:
            out = -10 + speed
        if speed + out < -10:
            out = -10 - speed
        elif speed + out > 10:
            out = 10 - speed
        get_line = True
        l_speed = speed - out
        r_speed = speed + out

def move():
    global get_line, r_speed, l_speed
    while True:
        if get_line:
            get_line = False
            vrep.simxSetJointTargetVelocity(clientID, left_motor, r_speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, right_motor, l_speed, vrep.simx_opmode_streaming)
        else:
            # vrep.simxSetJointTargetVelocity(clientID, left_motor, speed, vrep.simx_opmode_streaming)
            # vrep.simxSetJointTargetVelocity(clientID, right_motor, speed, vrep.simx_opmode_streaming)
            time.sleep(0.01)

th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

if clientID != -1:
    print('Connected to remote API server')
    _, body = vrep.simxGetObjectHandle(clientID, 'bubbleRob', vrep.simx_opmode_oneshot_wait)
    _, left_motor = vrep.simxGetObjectHandle(clientID, 'leftMotor', vrep.simx_opmode_oneshot_wait)
    _, right_motor = vrep.simxGetObjectHandle(clientID, 'rightMotor', vrep.simx_opmode_oneshot_wait)
    _, camera = vrep.simxGetObjectHandle(clientID, 'camera', vrep.simx_opmode_oneshot_wait)

    _, position = vrep.simxGetObjectPosition(clientID, body, -1, vrep.simx_opmode_streaming)
    _, orientation = vrep.simxGetObjectOrientation(clientID, body, -1, vrep.simx_opmode_streaming)
    time.sleep(0.5)

    # vrep.simxSetObjectPosition(clientID, body, -1, position, vrep.simx_opmode_oneshot)
    # vrep.simxSetObjectOrientation(clientID, body, -1, orientation, vrep.simx_opmode_oneshot)
    res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1):
        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
        if res == 0:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3])
            Tracing(img)
            cv2.imshow('img', img)
            cv2.waitKey(1)
    cv2.destroyAllWindows()