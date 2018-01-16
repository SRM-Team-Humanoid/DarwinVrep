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
    else:
        print "Connection Failed"
        sys.exit()

#errCode,motor1 = vrep.simxGetObjectHandle(ClientID,'j_tibia_r',vrep.simx_opmode_oneshot_wait)
#errCode,motor2 = vrep.simxGetObjectHandle(ClientID,'j_tibia_l',vrep.simx_opmode_oneshot_wait)
#vrep.simxSetJointTargetPosition(ClientID,motor2,-angle,vrep.simx_opmode_oneshot_wait)

def get_handles():
    global MOTOR_HANDLES
    jointNames={1:'j_shoulder_r',2:'j_shoulder_l',3:'j_high_arm_r',4:'j_high_arm_l',5:'j_low_arm_r',
                6:'j_low_arm_l',7:'j_pelvis_r',8:'j_pelvis_l',9:'j_thigh1_r',10:'j_thigh1_l',
                11:'j_thigh2_r',12:'j_thigh2_l',13:'j_tibia_r',14:'j_tibia_l',15:'j_ankle1_r',
                16:'j_ankle1_l',17:'j_ankle2_r',18:'j_ankle2_l',19:'j_pan',20:'j_tilt'} 
    for i in range(1,21):
        _,handle = vrep.simxGetObjectHandle(CLIENT_ID,jointNames[i],vrep.simx_opmode_oneshot_wait)
        MOTOR_HANDLES[i] = handle
 
def getPos():
        angles = []
        for i in MOTOR_HANDLES.keys():
            _,angle = vrep.simxGetJointPosition(CLIENT_ID,MOTOR_HANDLES[i],vrep.simx_opmode_oneshot_wait)
            angles.append(math.degrees(angle))
        print angles
        return Motion(1," ".join(map(str,angles)),0)
       



def setPos(writ):
    for key,val in writ.items():
        vrep.simxSetJointTargetPosition(CLIENT_ID,MOTOR_HANDLES[key],math.radians(val),vrep.simx_opmode_oneshot_wait)

class XmlTree(object):

    def __init__(self,str):
        try:
            with open(str) as f:
                self.tree = ET.ElementTree(file=str)
        except:
            print os.listdir('.')
            raise RuntimeError("File not found.")

    def parsexml(self,text):
        find = "PageRoot/Page[@name='" + text + "']/steps/step"
        motions = []
        prev_frame = 0
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            # print find
            raise RuntimeError("ParseFail!")
        for step in steps:
            motion = Motion(step.attrib['frame'], step.attrib['pose'], prev_frame)
            prev_frame = step.attrib['frame']
            motions.append(motion)

        return motions

    def superparsexml(self,text,exclude=[],offsets=[]):
        find = "FlowRoot/Flow[@name='"+text+"']/units/unit"
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            # print find
            raise RuntimeError("ParseFail!")
        motionsets = []
        for step in steps:
            motionsets.append(MotionSet(self.parsexml(step.attrib['main']),speed=float(step.attrib['mainSpeed']),exclude=exclude,offsets=offsets))

        return motionsets



class Motion(object):
    def __init__(self,frame,pose,prev_frame):
        self.frame = int(frame)
        self.pose = {}
        self.delay = self.frame-int(prev_frame)
        for i,p in enumerate(pose.split()):
            self.pose[i+1] =float(p)

    def __str__(self):
        return "Frame:"+str(self.frame) + "      Delay:"+str(self.delay) + "     Pose:"+" ".join(map(str,self.pose.values()))

    def updatePose(self,offset,add=True):
        if add:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] += offset[k]
        else:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] -= offset[k]


    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            try:
                begpos[k]
            except:
                begpos[k]=0
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            setPos(writ)
            print "k"
            time.sleep(0.008 / speed)
            # print writ



class MotionSet(object):
    def __init__(self,motions,speed=1.0,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        self.offsets = offsets
        self.loaded = False

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def stateUpdater(self,motion):
        global state
        for k in motion.pose.keys():
            state.pose[k]=motion.pose[k]

    def execute(self,speed=-1,iter=1):
        if speed<0:
            speed = self.speed
        if not self.loaded:
            for offset in self.offsets:
                for motion in self.motions:
                    motion.updatePose(offset)
            self.loaded = True

        while iter>0:
            for motion in self.motions:
                motion.write(state,speed,self.exclude)
                self.stateUpdater(motion)
            iter-=1

class Action():
    def __init__(self,motionsets):
        self.motionsets=motionsets

    def add(self,motionsets):
        self.motionsets.extend(motionsets)

    def execute(self,iter=1,speed=1):
        while iter>0:
            for motionset in self.motionsets:
                # for m in motionset.motions:
                #     print m
                orig = motionset.speed
                motionset.speed = motionset.speed*speed
                motionset.execute()
                motionset.speed = orig
            iter -= 1

#---------------------------
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
tree = XmlTree('data.xml')
balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin])
#kick = MotionSet(tree.parsexml("18 L kick"),speed=2,offsets=[darwin])
#w1 = MotionSet(tree.parsexml("32 F_S_L"),speed=2.1,offsets=[darwin])
#w2 = MotionSet(tree.parsexml("33 "),speed=2.1,offsets=[darwin])
#w3 = MotionSet(tree.parsexml("38 F_M_R"),speed=2.7,offsets=[darwin])
#w4 = MotionSet(tree.parsexml("39 "),speed=2.1,offsets=[darwin])
#w5 = MotionSet(tree.parsexml("36 F_M_L"),speed=2.7,offsets=[darwin])
#w6 = MotionSet(tree.parsexml("37 "),speed=2.1,offsets=[darwin])
#walk_init = Action([w1,w2])
#walk_motion = Action([w3,w4,w5,w6])
#--------------------------


connectVrep()
get_handles()
state = getPos()
raw_input(">")
balance.execute()

