import vrep
import sys
import math
import time

#vrep.simxfinish(-1)
ClientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if ClientID!=-1:
    print "Connected to V-Rep Server"
else:
    print "Connection Failed"
    sys.exit()

errCode,motor1 = vrep.simxGetObjectHandle(ClientID,'j_tibia_r',vrep.simx_opmode_oneshot_wait)
errCode,motor2 = vrep.simxGetObjectHandle(ClientID,'j_tibia_l',vrep.simx_opmode_oneshot_wait)

print motor1
print motor2


while True:
    for i in range(90):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor1,angle,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(ClientID,motor2,-angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)
    for i in range(90,-90,-1):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor1,angle,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(ClientID,motor2,-angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)
    for i in  range(-90,0):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor1,angle,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(ClientID,motor2,-angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)

