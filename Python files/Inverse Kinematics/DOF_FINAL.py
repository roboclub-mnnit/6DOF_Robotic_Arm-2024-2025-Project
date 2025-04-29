import numpy as np
import math


data=[[20, 20, 0, 0, 0, 0],
        [5, 1, 1, 25, 30, 45],
        [4,10,1,0,0,0],
        [4,10,1,39,21,130],
        [9.5,12.7,3,0,0,0],
        [9.5,12.7,3,87,98,17],
        [4,2.7,14,28,82,19],
        [12,2,12.1,29,99,12],
        [9,1,10,10,10,10],
        [5,5,5,5,5,5]]
data=np.array(data)
print( "given data =","\n",data,"\n")
i=0
# link length 
d1=8.0
d2=0.0
d3=11.0
d4=18.0
d5=5.5

d6=0


def cosinelaw(a,b,c):
     return np.arccos((a*a + b*b - c*c) / (2*a*b))

#----------------------------------------------------completed-------------------------------

# def ROTATION_MATRIX(roll,pitch,yaw):
#     R0_6_initial=np.matrix([
#         [0,0,1],
#         [0,-1,0],
#         [1,0,0]
#     ])
#     R_6_rotation=np.matrix([
#         [np.cos(pitch)*np.cos(roll),np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(pitch)*np.cos(yaw),np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)*np.cos(pitch)],
#         [np.sin(roll),np.cos(roll)*np.cos(yaw),-np.sin(yaw)*np.cos(roll)],
#         [-np.sin(pitch)*np.cos(roll),np.sin(pitch)*np.sin(roll)*np.cos(yaw)+np.sin(yaw)*np.cos(pitch),-np.sin(pitch)*np.sin(roll)*np.sin(yaw)+np.cos(pitch)*np.cos(yaw)]
#     ])

#     R0_6=np.dot(R0_6_initial,R_6_rotation)
#     print("print ro6",R0_6)
#     return R0_6

def ROTATION_MATRIX(roll,pitch,yaw):
    R0_6_initial=np.matrix([
        [0,0,1],
        [0,-1,0],
        [1,0,0]
    ])
    R_6_rotation=np.matrix([
        [np.cos(pitch)*np.cos(roll),np.sin(pitch)*np.sin(yaw)-np.sin(roll)*np.cos(pitch)*np.cos(yaw),np.sin(pitch)*np.cos(yaw)+np.sin(roll)*np.sin(yaw)*np.cos(pitch)],
        [np.sin(roll),np.cos(roll)*np.cos(yaw),-np.sin(yaw)*np.cos(roll)],
        [-np.sin(pitch)*np.cos(roll),np.sin(pitch)*np.sin(roll)*np.cos(yaw)+np.sin(yaw)*np.cos(pitch),-np.sin(pitch)*np.sin(roll)*np.sin(yaw)+np.cos(pitch)*np.cos(yaw)]
    ])

    R0_6=np.dot(R0_6_initial,R_6_rotation)
   #print("dj",R0_6)
    return R0_6

#----------------------------------------------------completed-------------------------------


def GRIPPER_COORDINATE(P0_6,d5,R0_6):
     # position vector of the wrist centre (P0_3),wrist position depend on first three joints
    d10=[[0],[0],[d5]]
    P0_6=np.transpose(P0_6)


    P0_3=P0_6-np.dot(R0_6,d10)
    #print(P0_3,"\n")


    # first problem is orientation problem.from this we get(theat 4,thata 5,thata 6),
    # second problem is position problem.from this we get(theat 1,thata 2,thata 3)
    # here wrist centre is treted as a end effector for first case
 
    # x-coordinate of the wrist centre
    # Access the element directly using array indexing, as P0_3 may be a matrix
    x = P0_3[0, 0]  
    # y-coordinate of the wrist centre
    y = P0_3[1, 0]  
    # z-coordinate of the wrist centre
    z = P0_3[2, 0]  
    print("xyz",x,y,z)
    return x,y,z

#----------------------------------------------------completed-------------------------------



def INVERSE_KINAMTICS_FIRST_3(x,y,z):
    #base of the wrist centre
    r=np.sqrt(x**2+y**2)

    #d is the distance from second joint to wrist centre in z direction
    s=z-d1

    #d is the displacement from second joint to wrist centre
    r2=np.sqrt((r-d2)**2+s**2)


        # calculation of theta 1,thata 2,theta 3
        # theta 1
    theta1=np.arctan2(y,x)

      # theta 3 
    phi3=cosinelaw(d3,d4,r2)
    theta3=phi3-np.pi

      # theta2 calculation:
    phi1=np.arctan(s/(r-d2))
    phi2=cosinelaw(d3,r2,d4)
    theta2 = phi1+phi2 
    print("theta 1 =",np.degrees(theta1),"\n")
    print("theta 2 = ",np.degrees(theta2),"\n")
    print("theta 3 = ",np.degrees(theta3),"\n")
    
    return theta1,theta2,theta3

#----------------------------------------------------completed-------------------------------

def INVERSE_KINAMTICS_LAST_3(theta1,theta2,theta3):

    def rotjoint1(theta1):
      """Rotation matrix around the z-axis."""
      M1=[[1,0,0],[0,0,-1],[0,1,0]]
      M1=np.array(M1)
      Rz=[[math.cos(theta1),-math.sin(theta1),0],
            [math.sin(theta1), math.cos(theta1),0],
            [0,0,1]]
                      
      return np.dot(Rz,M1)

    def rotjoint2(theta2):
      """Rotation matrix around the y-axis."""
      M1=[[1,0,0],[0,1,0],[0,0,1]]
      M1=np.array(M1)
      Rz=[[np.cos(theta2),-np.sin(theta2),0],
          [np.sin(theta2),np.cos(theta2),0],
           [0,0,1]]
      return np.dot(Rz,M1)

    def rotjoint3(theta3):
      """Rotation matrix around the x-axis."""
      M1=[[0,0,1],[1,0,0],[0,1,0]]
      M1=np.array(M1)
      Rz=[[np.cos(theta3),-np.sin(theta3),0],
          [np.sin(theta3),np.cos(theta3),0],
           [0,0,1]]
      return np.dot(Rz,M1)

    # Calculate R0_3 using individual rotation matrices
    R0_1 = rotjoint1(theta1)
    R1_2 = rotjoint2(theta2)
    R2_3 = rotjoint3(theta3)

    R0_3 = np.dot(R0_1, np.dot(R1_2, R2_3))

# def Rz(theta):
#     return np.array([
#         [np.cos(theta), -np.sin(theta), 0],
#         [np.sin(theta),  np.cos(theta), 0],
#         [0,              0,             1]
#     ])

# def Rx(alpha):
#     return np.array([
#         [1, 0,              0],
#         [0, np.cos(alpha), -np.sin(alpha)],
#         [0, np.sin(alpha),  np.cos(alpha)]
#     ])

# # Assume theta1, theta2, theta3 are computed from the inverse kinematics for the first three joints.
# # They should be in radians.
# def INVERSE_KINAMTICS_LAST_3(theta1, theta2, theta3):
#     # DH parameters for links 1,2,3: α1 = π/2, α2 = 0, and for link3, note the offset (q3 + π/2 and α3 = π/2)
#     R0_1 = np.dot(Rz(theta1), Rx(np.pi/2))
#     R1_2 = Rz(theta2)
#     R2_3 = np.dot(Rz(theta3 + np.pi/2), Rx(np.pi/2))
    
#     R0_3 = np.dot(R0_1, np.dot(R1_2, R2_3))
#     # return R0_3


    print(R0_3)
    #print(R0_6)
    R0_3inv=np.linalg.inv(R0_3)
    #print("hi",R0_3inv)
    # Calculate R3_6
    
    R3_6 = np.dot(R0_3inv, R0_6)  # Corrected: Use np.dot and R0_3_inv        
    # print ("dc",R3_6)
   
                                                      

    # calculation of theta4,theta5,theta6

    #theta 5
    theta5=np.arccos(R3_6[2,2])

   #theta 6

    theta6=np.arcsin(R3_6[2,1]/np.sin(theta5))
  #theta 4
    theta4=np.arcsin(R3_6[1,2]/np.sin(theta5))

    print("theta 4 =  ",np.degrees(theta4),"\n") 
    print("theta 5 = ",np.degrees(theta5),"\n")
    print("theta 6 = ", np.degrees(theta6),"\n")






# position vector of the end effector 0P6,end effector position depend on last three joint
for i in range(10):

    print("angle of the joints for given data for ",i+1," case =")
    
    P0_6=data[i:i+1,0:3]

    P0_10=data[:i+1,3:6]

    print(P0_6,"\n")
   # print(P0_10,"n")


    # rotation matrix from roll,pitch,yaw (R0_6)
    #import math



    roll = np.radians(P0_10[i][0])
    pitch =np.radians( P0_10[i][1])
    yaw = np.radians(P0_10[i][2])

    R0_6 =  ROTATION_MATRIX(roll,pitch,yaw)

    # gripper coordinate
    x,y,z =  GRIPPER_COORDINATE(P0_6,d5,R0_6)

    # first 3 joint angle
    theta1,theta2,theta3 = INVERSE_KINAMTICS_FIRST_3(x,y,z)

    # last 3 joint angles()
    INVERSE_KINAMTICS_LAST_3(theta1,theta2,theta3)

    

    

    
   