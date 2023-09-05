import sys
import os
import math 

os.chdir("../")
path1 = os.getcwd()

os.chdir("coppeliasim_utils")
path2 = os.getcwd()


sys.path.append(path1)
sys.path.append(path2)

from indy_utils import indydcp_client as client
from coppeliasim_utils import sim

from time import sleep
import time
import json
import threading
import numpy as np
import math
import keyboard
import threading

robot_ip = "192.168.3.19" 
#robot_ip = "192.168.120.163"  # Robot (Indy) IP
robot_name = "NRMK-Indy7"  # Robot name (Indy7)

# Create class object (real robot)
indy_actual = client.IndyDCPClient(robot_ip, robot_name)

# Crate class object (simulation robot)
indy_virtual = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

# Actual robot (Created: True, Not created: False)
# Virtual robot (Created: True, Not created: False)
print("real robot: {0}\nsimulation robot: {1}".format(bool(indy_actual), bool(not indy_virtual)))

# Reset TCP-IP communication (real robot)
# For real robot
indy_actual.connect()
indy_actual.disconnect()


# For simulation robot
sim.simxFinish(indy_virtual)

# For real robot
indy_actual.connect()
j_pos_act = indy_actual.get_joint_pos()
t_pos_act = indy_actual.get_task_pos()

# For simulation robot
handles = []
handles2 = []
j_pos_vir = []
t_pos_vir = []
d_pos_vir = []
print( sim.simx_return_ok)

for i in range(6):
    object_name = 'joint' + str(i)
    result, handle=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object handle for first joint')                
    else:
        handles.append(handle)
        result, jnt_angle = sim.simxGetJointPosition(indy_virtual, handle, sim.simx_opmode_oneshot)
        j_pos_vir.append(jnt_angle/math.pi*180)
        
object_name = 'tool_coordinate'
result, handle=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)

result, obj_pos = sim.simxGetObjectPosition(indy_virtual, handle, -1, sim.simx_opmode_oneshot)
result, obj_rot = sim.simxGetObjectOrientation(indy_virtual, handle, -1, sim.simx_opmode_oneshot)

object_name = 'doorhand'
result2, handle2=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)

result2, obj_pos2 = sim.simxGetObjectPosition(indy_virtual, handle2, -1, sim.simx_opmode_oneshot)
result2, obj_rot2 = sim.simxGetObjectOrientation(indy_virtual, handle2, -1, sim.simx_opmode_oneshot)

#그리퍼
handles_grip = []
#hFeederJoint ={-1,-1,-1}
       
for i in range(2):
    object_name = 'gripper_joint' + str(i)
    result3, handle3=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)
    if result3 != sim.simx_return_ok:
        raise Exception('could not get object handle for first joint')                
    else:
        handles2.append(handle)
        result3, jnt_angle3 = sim.simxGetJointPosition(indy_virtual, handle3, sim.simx_opmode_oneshot)
        j_pos_vir3.append(jnt_angle3/math.pi*180)        
    
########################    
print()

for i in range(3):
    t_pos_vir.append(obj_pos[i])

for i in range(3):
    t_pos_vir.append(obj_rot[i]/np.pi*180)

print("Joint Position (real):", np.round(j_pos_act, 2))
print("Joint Position (simulation):", np.round(j_pos_vir, 2))

print()
print("Task Position (real):", np.round(t_pos_act,2))
print("Task Position (simulation):", np.round(t_pos_vir,2))

t_pos_vir.clear()

for i in range(3):
    t_pos_vir.append(obj_pos2[i])

for i in range(3):
    t_pos_vir.append(obj_rot2[i]/np.pi*180)

print()
print("Door_handle Position (sim):", np.round(t_pos_vir,2))

print("Gripper Joint Position (simulation):", np.round(j_pos_vir3, 2))

### For actual robot
indy_actual.connect()

# Zero position
#j_pos_act = [0, 0, 0, 0, 0, 0]

# Home position
j_pos_act = [0, -15, -90, 0, -75, 0]
indy_actual.joint_move_to(j_pos_act)  # Move 6th joint

indy_actual.disconnect()

### For virtual robot
handles = []
#Zero position
#j_pos_vir = [0, 0, 0, 0, 0, 0]

# Home position
j_pos_vir = [0, -15, -90, 0, -75, 0]

for i in range(6):
    object_name = 'joint' + str(i)
    #print(object_name)
    result, handle=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object handle for first joint')                
    else:
        handles.append(handle)
        sim.simxSetJointTargetPosition(indy_virtual, handle, j_pos_vir[i]*math.pi/180, sim.simx_opmode_oneshot)

def cps_program():
    indy_actual.connect()

    handles = []

    for i in range(6):
            object_name = 'joint' + str(i)    
            #print(object_name)
            result, handle=sim.simxGetObjectHandle(indy_virtual, object_name, sim.simx_opmode_blocking)
            handles.append(handle)

    print(handles)

    isCtrOn = True

    while isCtrOn:
        j_pos = indy_actual.get_joint_pos()
        time.sleep(0.002)

        for i in range(6):
            sim.simxSetJointTargetPosition(indy_virtual, handles[i] , float(j_pos[i])*math.pi/180 ,sim.simx_opmode_oneshot)

        if keyboard.is_pressed("q"):
            isCtrOn = False
            print("Escape the program")

    indy_actual.disconnect()

t = threading.Thread(target=cps_program)
t.start()
print("Main Thread")


j_pos1 = indy_actual.get_joint_pos()
t_pos1 = indy_actual.get_task_pos()
print("j_pos1", j_pos1)
print("t_pos1", t_pos1)

#t_pos1[2] += 0.1

#m미터

a = math.pi/180
print("a : ", a)
pos1 = [ -0, -15, -90, 0, 4.12, 0]
pos2 = [ -0, -34.54, -73.35, 0, 4.12, 0]
pos3 = [t_pos_vir[0], t_pos_vir[1], t_pos_vir[2], t_pos_vir[3], t_pos_vir[4], t_pos_vir[5]]
pos3_j = [89.95, 0.05, -44.13, 9.43, -67.14,-7] #문을 향해 가는 거 조인트
#indy_actual.task_move_to(t_pos1)  # Move along z-axis
indy_actual.joint_move_to(pos1) 
sleep(4)
indy_actual.joint_move_to(pos3) 


#indy_actual.joint_move_to(pos3_j) 
#indy_actual.get_inv_kin(pos3,pos1)

#sleep(4)
