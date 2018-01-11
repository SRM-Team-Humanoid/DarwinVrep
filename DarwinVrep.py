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

errCode,motor = vrep.simxGetObjectHandle(ClientID,'j_pan',vrep.simx_opmode_oneshot_wait)

print motor


while True:
    for i in range(90):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor,angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)
    for i in range(90,-90,-1):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor,angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)
    for i in  range(-90,0):
        angle = math.radians(i)
        vrep.simxSetJointTargetPosition(ClientID,motor,angle,vrep.simx_opmode_oneshot_wait)
        time.sleep(0.001)

