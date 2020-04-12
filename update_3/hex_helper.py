import vrep
import time
import numpy as np

#######################################################################################
# Function to grab all the handles of the joints and leg tips and body from CoppeliaSim
#######################################################################################
def init_handles(clientID):
    body_handles = [0,0,0,0,0,0,0]
    handles = [[handle_grabber(i,j,clientID) for i in range(3)] for j in range(6)]
    
    result, body_handles[6] = vrep.simxGetObjectHandle(clientID, 'hexa_body', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception(f'could not get object handle for hexa body')
        
    for i in range(6):
        result, body_handles[i] = vrep.simxGetObjectHandle(clientID, f'hexa_footTip{i}', vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception(f'could not get object handle for foot tip {i+1} ')
    print(handles,body_handles)
    return handles, body_handles

# Helper function for init_handles
def handle_grabber(i,j,clientID):
    result, handle = vrep.simxGetObjectHandle(clientID, f'hexa_joint{i+1}_{j}', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception(f'could not get object handle for joint {i+1} of leg {j}')
    return handle

################################################################################################
# Helper function to return the coords of joints (so that the segment lengths can be determined)
################################################################################################
def get_joint(handles, body_handles, clientID):
    X = []
    Y = []
    Z = []
    result,vector=vrep.simxGetObjectPosition(clientID, handles[0][0],body_handles[6],vrep.simx_opmode_blocking)
    X.append(vector[0])
    Y.append(vector[1])
    Z.append(vector[2])
    result,vector=vrep.simxGetObjectPosition(clientID, handles[0][1],body_handles[6],vrep.simx_opmode_blocking)
    X.append(vector[0])
    Y.append(vector[1])
    Z.append(vector[2])
    result,vector=vrep.simxGetObjectPosition(clientID, handles[0][2],body_handles[6],vrep.simx_opmode_blocking)
    X.append(vector[0])
    Y.append(vector[1])
    Z.append(vector[2])
    result,vector=vrep.simxGetObjectPosition(clientID, body_handles[0],body_handles[6],vrep.simx_opmode_blocking)
    X.append(vector[0])
    Y.append(vector[1])
    Z.append(vector[2])

#     X = np.round(X, decimals = 6)
#     Y = np.round(Y, decimals = 6)
#     Z = np.round(Z, decimals = 6)
    return X,Y,Z