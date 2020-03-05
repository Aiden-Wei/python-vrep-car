import vrep
import sys
import numpy as np
import cv2
import time
import math

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')

    _, body = vrep.simxGetObjectHandle(clientID, 'bubbleRob', vrep.simx_opmode_oneshot_wait)
    # _, leftmotor = vrep.simxGetObjectHandle(clientID, 'leftMotor', vrep.simx_opmode_oneshot_wait)
    # _, rightmotor = vrep.simxGetObjectHandle(clientID, 'rightMotor', vrep.simx_opmode_oneshot_wait)

    # returnCode = vrep.simxSetObjectPosition(clientID, body, -1, [0.6, 0, 0.12], vrep.simx_opmode_oneshot)
    returnCode = vrep.simxSetObjectOrientation(clientID, body, -1, [0, 0, math.pi/3], vrep.simx_opmode_oneshot)
    returnCode = vrep.simxSetObjectPosition(clientID, body, -1, [0.6, 0, 0.12], vrep.simx_opmode_oneshot)
    time.sleep(1)
    returnCode = vrep.simxSetObjectOrientation(clientID, body, -1, [0, 0, 0], vrep.simx_opmode_oneshot)
    returnCode = vrep.simxSetObjectPosition(clientID, body, -1, [0, 0, 0.12], vrep.simx_opmode_oneshot)
    # while True:
    #     if returnCode == 1:
    #         returnCode = vrep.simxSetObjectPosition(clientID, body, -1, [0.6, 0, 0.12], vrep.simx_opmode_oneshot)
    #     elif returnCode == 0:
    #         returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID, body, -1, vrep.simx_opmode_oneshot)
    #         if returnCode == 0:
    #             print(eulerAngles)
    #             break
    # floorSensorHandles=[-1, -1, -1]
    # _, floorSensorHandles[0] = vrep.simxGetObjectHandle(clientID, 'leftSensor', vrep.simx_opmode_oneshot_wait)
    # _, floorSensorHandles[1] = vrep.simxGetObjectHandle(clientID, 'middleSensor', vrep.simx_opmode_oneshot_wait)
    # _, floorSensorHandles[2] = vrep.simxGetObjectHandle(clientID, 'rightSensor', vrep.simx_opmode_oneshot_wait)
    # for i in range(3):
    #     returnCode, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, floorSensorHandles[i], vrep.simx_opmode_streaming)
    #
    # # _, camera = vrep.simxGetObjectHandle(clientID, 'cam1', vrep.simx_opmode_oneshot_wait)
    # # res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)
    # while (vrep.simxGetConnectionId(clientID) != -1):
    #     sensorReading = [False, False, False]
    #     for i in range(0, 3):
    #         returnCode, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, floorSensorHandles[i], vrep.simx_opmode_buffer)
    #         if returnCode == 0:
    #             sensorReading[i] = (auxPackets[0][11] < 0.3)
    #     if sensorReading[0]:
    #         vrep.simxSetJointTargetVelocity(clientID, leftmotor, 0, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID, rightmotor, 2, vrep.simx_opmode_streaming)
    #     elif sensorReading[2]:
    #         vrep.simxSetJointTargetVelocity(clientID, leftmotor, 2, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID, rightmotor, 0, vrep.simx_opmode_streaming)
    #     else:
    #         vrep.simxSetJointTargetVelocity(clientID, leftmotor, 1, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID, rightmotor, 1, vrep.simx_opmode_streaming)

        # vrep.simxSetJointTargetVelocity(clientID, leftSensor, 0.2, vrep.simx_opmode_streaming)
        # vrep.simxSetJointTargetVelocity(clientID, middleSensor, 0.2, vrep.simx_opmode_streaming)
        # res, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
        # if res == 0:
        #     # print(res, image)
        #     img = np.array(image, dtype=np.uint8)
        #     img.resize([resolution[0], resolution[1], 3])
        #     imgflip = cv2.flip(img, 0)
        #     cv2.imshow('1', imgflip)
        #     cv2.waitKey(1)

