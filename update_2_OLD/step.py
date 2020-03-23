import vrep
import time
from copy import copy as copy

def move_leg(thetas,leg):
    vrep.simxSetJointTargetPosition(clientID, handles[leg][0], thetas[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][1], thetas[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][2], thetas[2], vrep.simx_opmode_oneshot)