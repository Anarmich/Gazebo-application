#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab2_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

p1 = [-.150, .150, .010]
p2 = [-.150, .270, .162]
p3 = [.094, .270, .162]
p4 = [.307, .177, .162]
p5 = [.307, .260, .162]
p6 = [.390, .260, .162]
pM = [.390, .401, .215]
SBracket = np.zeros((4,4,6))

def Get_MS():
    # =================== Your code starts here ====================#
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))

    global p1
    global p2
    global p3
    global p4
    global p5
    global p6
    global pM

    w1 = [0,0,1]
    v1 = np.cross(np.negative(w1),p1)
    w2 = [0,1,0]
    v2 = np.cross(np.negative(w2),p2)
    w3 = [0,1,0]
    v3 = np.cross(np.negative(w3),p3)
    w4 = [0,1,0]
    v4 = np.cross(np.negative(w4),p4)
    w5 = [1,0,0]
    v5 = np.cross(np.negative(w5),p5)
    w6 = [0,1,0]
    v6 = np.cross(np.negative(w6),p6)

    M = [[0,-1,0,pM[0]],[0,0,-1,pM[1]],[1,0,0,pM[2]],[0,0,0,1]]

    S1=np.r_[w1,v1]
    S2=np.r_[w2,v2]
    S3=np.r_[w3,v3]
    S4=np.r_[w4,v4]
    S5=np.r_[w5,v5]
    S6=np.r_[w6,v6]
   
    S=np.vstack([[S1], [S2], [S3], [S4], [S5], [S6]])

    # ==============================================================#
    return M, S

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    # Initialize the return_value
    return_value = [None, None, None, None, None, None]

    #print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    global SBracket
    S = Get_MS()[1]
    M = Get_MS()[0]
    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]

    SBracket = np.zeros((4,4,6))
    for i in range(6):
        SBracket[:,:,i] = [[0, -S[i][2], S[i][1], S[i][3]], \
                      [S[i][2], 0, -S[i][0], S[i][4]], \
                      [-S[i][1], S[i][0], 0, S[i][5]], \
                      [0, 0, 0, 0]]

    T06 = np.ones((4,4))

    T01 = np.matmul(expm(SBracket[:,:,0]*thetas[0]),expm(SBracket[:,:,1]*thetas[1]))
    T02 = np.matmul(T01,expm(SBracket[:,:,2]*thetas[2]))
    T03 = np.matmul(T02,expm(SBracket[:,:,3]*thetas[3]))
    T04 = np.matmul(T03,expm(SBracket[:,:,4]*thetas[4]))
    T05 = np.matmul(T04,expm(SBracket[:,:,5]*thetas[5]))
    T06 = np.matmul(T05,M)

    #print(str(T06) + "\n") #STOPPED THE PRINT FORWARD KIN
    # ==============================================================#

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # =================== Your code starts here ====================#

# The Givens
    yaw = np.radians(yaw_WgripDegree)

    Tbw = [[1, 0, 0, 0.15],\
           [0, 1, 0, -0.15],\
           [0, 0, 1, -0.01],\
           [0, 0, 0, 1]]
    pgrip = [[xWgrip],[yWgrip],[zWgrip],[1]]
    pbgrip = np.matmul(Tbw,pgrip)
    x = pbgrip[0][0]
    y = pbgrip[1][0]
    z = pbgrip[2][0]
    #print(str(pbgrip) + "\n")

# The Knowns
    theta5 = np.radians(-90)
    L1 = 0.152
    L3 = 0.244
    L5 = 0.213
    L6 = 0.083
    L7 = 0.083
    L8 = 0.082
    L9 = 0.0535
    L10 = 0.059

# 1
    xcen = x - np.cos(yaw)*L9
    ycen = y - np.sin(yaw)*L9
    zcen = z
    #print(str(xcen) + "\n")
    #print(str(ycen) + "\n")    

# 2
    Lcen = np.sqrt(xcen**2 + ycen**2)
    theta1 = np.arctan2(ycen,xcen) - np.arcsin((L6 + 0.027)/Lcen)

#3
    theta6 = PI/2 - yaw + theta1

#4
    pc3end = [[-L7],[-(.027 + L6)],[L8 + L10],[1]]
    pbc = [xcen,ycen,zcen]
    #print(str(pbc) + "\n")
    Rbc = [[np.cos(theta1), -np.sin(theta1), 0],\
           [np.sin(theta1), np.cos(theta1), 0],\
           [0, 0, 1]]

    Tbc = [[Rbc[0][0], Rbc[0][1], Rbc[0][2], pbc[0]],\
           [Rbc[1][0], Rbc[1][1], Rbc[1][2], pbc[1]],\
           [Rbc[2][0], Rbc[2][1], Rbc[2][2], pbc[2]],\
           [0, 0, 0, 1]]
    pb3end = np.matmul(Tbc,pc3end)
    #print(str(pb3end) + "\n")
    x3end = pb3end[0][0]
    y3end = pb3end[1][0]
    z3end = pb3end[2][0]

#5
    L = np.sqrt(x3end**2 + y3end**2)
    gamma = np.sqrt(L**2 + (z3end - L1)**2)
    B = np.arccos(L/gamma)

    theta2 = -(np.arccos((-L5**2 + L3**2 + gamma**2)/(2*L3*gamma)) + B)
    theta4 = -(np.arccos((-L3**2+gamma**2+L5**2)/(2*gamma*L5)) - B)
    theta3 = -theta4 - theta2
    #theta3 = PI - np.arccos((gamma**2 - L5**2 - L3**2)/(-2*L5*L3))
    #theta4 = -(theta3 - (-theta2))

    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    thetadegrees = np.degrees(thetas)

    #forward = lab_fk(thetadegrees[0], thetadegrees[1], thetadegrees[2], thetadegrees[3], thetadegrees[4], thetadegrees[5])
    forward = lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    #print(str(thetadegrees) + "\n")
    #print(str(thetas) + "\n")

    return forward
    # ==============================================================#
    pass
