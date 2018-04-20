from lib import vrep
import sys
import math
import time
import os
import xml.etree.cElementTree as ET
import numpy as np


#vrep.simxfinish(-1)

CLIENT_ID = None
MOTOR_HANDLES = {}

def connectVrep():
    global CLIENT_ID
    CLIENT_ID= vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if CLIENT_ID!=-1:
        print "Connected to V-Rep Server"
        res,objs = vrep.simxGetObjects(CLIENT_ID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
        if res == vrep.simx_return_ok:
            print "No. of Objects = ",len(objs)
    else:
        print "Connection Failed"
        sys.exit()

#errCode,motor1 = vrep.simxGetObjectHandle(ClientID,'j_tibia_r',vrep.simx_opmode_oneshot_wait)
#angle = 120
#errCode,motor2 = vrep.simxGetObjectHandle(ClientID,'j_tibia_l',vrep.simx_opmode_oneshot_wait)
#vrep.simxSetJointTargetPosition(ClientID,motor2,-angle,vrep.simx_opmode_oneshot_wait)

def get_handles():
    global MOTOR_HANDLES
    jointNames={1:'j_shoulder_r',2:'j_shoulder_l',3:'j_high_arm_r',4:'j_high_arm_l',5:'j_low_arm_r',
                6:'j_low_arm_l',7:'j_pelvis_r',8:'j_pelvis_l',9:'j_thigh1_r',10:'j_thigh1_l',
                11:'j_thigh2_r',12:'j_thigh2_l',13:'j_tibia_r',14:'j_tibia_l',15:'j_ankle1_r',
                16:'j_ankle1_l',17:'j_ankle2_r',18:'j_ankle2_l',19:'j_pan',20:'j_tilt'} 
    for i in range(1,21):
        e,handle = vrep.simxGetObjectHandle(CLIENT_ID,jointNames[i],vrep.simx_opmode_oneshot_wait)
        if e!=0:
            print "Error  = ",e
            exit(1)
        MOTOR_HANDLES[i] = handle
 
def getPos():
        angles = []
        for i in MOTOR_HANDLES.keys():
            _,angle = vrep.simxGetJointPosition(CLIENT_ID,MOTOR_HANDLES[i],vrep.simx_opmode_oneshot_wait)
            angles.append(math.degrees(angle))
        return angles
       



def setPos(writ):
    for key,val in writ.items():
        vrep.simxSetJointTargetPosition(CLIENT_ID,MOTOR_HANDLES[key],math.radians(val),vrep.simx_opmode_oneshot_wait)

connectVrep()
get_handles()
state = getPos()
print(state)
angle = 5
errCode,motor2 =vrep.simxGetObjectHandle(CLIENT_ID,'j_shoulder_r',vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(CLIENT_ID,motor2,-angle,vrep.simx_opmode_oneshot)
time.sleep(3)
state = getPos()
print(state)


