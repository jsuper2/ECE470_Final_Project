import vrep
import time
import numpy as np
from numpy import cos as cos
from numpy import sin as sin
from numpy import arctan as atan
from numpy import arccos as acos
from numpy import arcsin as asin
from copy import copy as copy

num_legs = 6

step_radius = 0.04
step_height = 0.03
x_offset = 0.00
y_offset = -0.11
z_offset = -0.08

num_top_points = 16
num_bot_points = 20
num_pts = num_top_points + num_bot_points

# helper rotation matrix functions
# def R_z(k,vector):
#      phi = -np.pi/3
#      rotm = np.array([[cos(k*phi),sin(k*phi),0],[-sin(k*phi),cos(k*phi),0],[0,0,1]])
#      return np.transpose(np.matmul(rotm,np.transpose(vector)))
def R_z(phi,vector):
    rotm = np.array([[cos(phi),sin(phi),0],[-sin(phi),cos(phi),0],[0,0,1]])
    return np.transpose(np.matmul(rotm,np.transpose(vector)))
def rotate_step_path(phi,sp_given):
    for j in range(len(sp_given[0,:])):
        sp_roted = np.transpose(np.array([R_z(phi,l) for l in np.transpose(sp_given)]))
    return sp_roted

# function to set all 3 joints in a leg
def move_leg(thetas,leg,clientID,handles):
    vrep.simxSetJointTargetPosition(clientID, handles[leg][0], thetas[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][1], thetas[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, handles[leg][2], thetas[2], vrep.simx_opmode_oneshot)

# Generates a joint angle matrix based on a desired translational and angular veocity
# vel should be a 1x3 numpy array the first two values are the x and y porportional
# velocities. The last number is the angular proportional velocity about the z axis
# all should be 0.0 to 1.0
def get_theta_matrix(direction, spin):
    # Setup the thetas used for the upper arc-section of the step
    theta = np.linspace(np.pi,0,num_top_points+2)[1:-1]
    # Setup the x,y,z coordinates of the foot path sections
    x = step_radius*np.linspace(1,-1,num_bot_points)
    x = np.append(x,step_radius*cos(theta))
    z = np.zeros(num_bot_points)
    z = np.append(z,step_height*sin(theta))
    y = np.zeros(num_bot_points+num_top_points)
    # Put them all together
    step_path = np.array([x,y,z])

    # Rotate to desired direction
    step_path = rotate_step_path(direction,step_path)
    
    # Create empty array of step paths
    step_paths = np.zeros([num_legs,3,num_pts])
    joint_angles = np.zeros([num_legs,3,num_pts])
    
    # Fill array of step paths through shifting and rotating
    step_paths[0,:,:] = step_path
    for i in range(num_legs-1):
        rollby = (i+1)%2*int(num_pts/2)
        step_paths[i+1,:,:] = np.roll(rotate_step_path(-(i+1)*np.pi/3,step_path),rollby,axis=1)
        
    # Translate all the steps + Make joint angle matrix
    for i in range(num_legs):
        for j in range(num_pts):
            step_paths[i,:,j] = step_paths[i,:,j]+np.array([x_offset,y_offset,z_offset])
            joint_angles[i,:,j] = leg_IK(step_paths[i,:,j])
    
    # Add spin to joint angle paths
    
    # Return those juicy sexy lovely joint angles
    return joint_angles

def step_3(thetas, clientID, handles):
    for i in range(num_pts):
        move_leg(thetas[0,:,i],0,clientID,handles)
        move_leg(thetas[1,:,i],1,clientID,handles)
        move_leg(thetas[2,:,i],2,clientID,handles)
        move_leg(thetas[3,:,i],3,clientID,handles)
        move_leg(thetas[4,:,i],4,clientID,handles)
        move_leg(thetas[5,:,i],5,clientID,handles)
        time.sleep(0.02)

##########################################################################
# Inverse Kin for a leg, returns joint angle for a given foot tip position
##########################################################################
# def leg_IK(pos):
#     x = pos[0]
#     y = pos[1]
#     z = pos[2]
#     theta1 = atan(x/abs(y))
    
#     theta2 = abs(atan(((x**2 + y**2)**(1/2) - 0.051496976497310442688615239603678)/z)) + \
#              acos((6.9078370030605173735242143743271*(((x**2 + y**2)**(1/2) - 0.051496976497310442688615239603678)**2 + \
#              z**2 - 0.0083358815796936158726282428688137))/(((x**2 + y**2)**(1/2) - \
#              0.051496976497310442688615239603678)**2 + z**2)**(1/2)) - 1.5707963267948966192313216916398
    
#     theta3 = acos(1.115463010227933919641721202415 - 59.288794418631241235969768311301*z**2 - \
#              59.288794418631241235969768311301*((x**2 + y**2)**(1/2) - 0.051496976497310442688615239603678)**2)

# OLD OLD OLD OLD OLD
def leg_IK(pos):
    x = pos[0]
    y = pos[1]
    z = pos[2]

    Y4 = 0.11651167932822327
    Z4 = 0.006
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