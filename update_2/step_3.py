import vrep
import time
import numpy as np
from numpy import cos as cos
from numpy import sin as sin
from copy import copy as copy

center = np.array([ 0. , -0.11 , -0.08])
# edge = [ 0.05210302, -0.17757873 , 0.03120471]
theta_center = np.array([0,-0.52359878,2.0943951])
radius_max = 0.04

# initialize the matrix of step coordinates
coords = np.array([[center for i in range(4)]for j in range(6)])

# helper rotation matrix function
def rotato(i,vector):
     phi = np.pi/3
     rotm = np.array([[cos(i*phi),sin(i*phi),0],[-sin(i*phi),cos(i*phi),0],[0,0,1]])
     return np.transpose(np.matmul(rotm,np.transpose(vector)))

# function to set all 3 joints in a leg
def move_leg(thetas,leg,clientID,handles):
    vrep.simxSetJointTargetPosition(clientID, handles[leg][0], thetas[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][1], thetas[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][2], thetas[2], vrep.simx_opmode_oneshot)

# Generates a joint angle matrix based on a desired translational and angular veocity
# vel should be a 1x3 numpy array the first two values are the x and y porportional
# velocities. The last number is the angular proportional velocity about the z axis
# all should be 0.0 to 1.0
def get_theta_matrix(vel):
     coord_delta = copy(vel)*radius_max
     coord_delta[2] = 0
     # forward = center+coord_delta
     # backward = center-coord_delta

     for i in range(6):
          rot_delta = rotato(i,coord_delta)
          pt1 = center+rot_delta
          pt2 = copy(pt1)
          pt2[2] = pt2[2]+.03
          pt4 = center-rot_delta
          pt3 = copy(pt4)
          pt3[2] = pt3[2]+.03
          coords[i]=[pt1,pt2,pt3,pt4]
     # for i in range(6):
     #      new_step_low = rotato(i,first_step)
     #      print(new_step_low)
     #      new_step_high = new_step_low
     #      new_step_high[2] = new_step_high[2]+0.11
     #      print(new_step_high)

     #      coords[i][0] = new_step_low
     #      coords[i][1] = new_step_high
     #      coords[-(-i+3)][2] = new_step_high
     #      coords[-(-i+3)][3] = new_step_low

     thetas = [[leg_IK(coords[j][i]) for i in range(4)] for j in range(6)]
     return thetas
def step_3(thetas, clientID, handles):
    #print('step')
    # 
    move_leg(thetas[0][1],0,clientID,handles)
    move_leg(thetas[2][1],2,clientID,handles)
    move_leg(thetas[4][1],4,clientID,handles)
    move_leg(thetas[1][3],1,clientID,handles)
    move_leg(thetas[3][3],3,clientID,handles)
    move_leg(thetas[5][3],5,clientID,handles)
    time.sleep(.3)

    move_leg(thetas[0][2],0,clientID,handles)
    move_leg(thetas[2][2],2,clientID,handles)
    move_leg(thetas[4][2],4,clientID,handles)
    move_leg(thetas[1][0],1,clientID,handles)
    move_leg(thetas[3][0],3,clientID,handles)
    move_leg(thetas[5][0],5,clientID,handles)
    time.sleep(.3)

    move_leg(thetas[0][3],0,clientID,handles)
    move_leg(thetas[2][3],2,clientID,handles)
    move_leg(thetas[4][3],4,clientID,handles)
    move_leg(thetas[1][1],1,clientID,handles)
    move_leg(thetas[3][1],3,clientID,handles)
    move_leg(thetas[5][1],5,clientID,handles)
    time.sleep(.3)

    move_leg(thetas[0][0],0,clientID,handles)
    move_leg(thetas[2][0],2,clientID,handles)
    move_leg(thetas[4][0],4,clientID,handles)
    move_leg(thetas[1][2],1,clientID,handles)
    move_leg(thetas[3][2],3,clientID,handles)
    move_leg(thetas[5][2],5,clientID,handles)
    time.sleep(.3)


##########################################################################
# Inverse Kin for a leg, returns joint angle for a given foot tip position
##########################################################################
def leg_IK(pos):
    x = pos[0]
    y = pos[1]
    z = pos[2]

    Y4 = 0.11651167932822327
    Z4 = 0
    Y2 = 0.05149697649731044
    Y3 = 0.07238155732083357

    le = np.sqrt(Y4*Y4+Z4*Z4)
    
    theta1 = np.arctan(x/np.abs(y))
    L1 = np.sqrt(x*x+y*y)
    L2 = (L1-Y2)*(L1-Y2)
    L = np.sqrt(z*z+L2)
    alpha1 = np.abs(np.arctan((L1-Y2)/z))
    alpha2 = np.arccos((L*L+Y3*Y3-le*le)/(2*L*Y3))
    theta2 = alpha1+alpha2-np.pi/2
    beta = np.arccos((le*le+Y3*Y3-L*L)/(2*le*Y3))
    gamma = np.arctan(Z4/Y4)
    theta3 = beta-gamma
    return [theta1,-theta2,theta3*2]