from sympy import symbols, cos, sin, pi
from sympy.matrices import Matrix
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Link lengths
a1 = 5.2
a2 = 11.2
a3 = 13.2
a4 = 5.5
a5 = 4.2

def DH_Params_H_Matrix(i, j, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0):
    DH_params = np.array([
        [q1, np.pi / 2, a2, a1],
        [q2, 0, a3, 0],
        [q3 + np.pi / 2, np.pi / 2, 0, 0],
        [q4, -np.pi / 2, 0, a4],
        [q5, np.pi / 2, 0, 0],
        [q6, 0, 0, a5]
    ])
    H_matrix = Matrix(np.identity(4))
    for k in range(i, j):
        H2_matrix = Matrix([
            [np.cos(DH_params[k, 0]), -np.sin(DH_params[k, 0]) * np.cos(DH_params[k, 1]), np.sin(DH_params[k, 0]) * np.sin(DH_params[k, 1]), DH_params[k, 2] * np.cos(DH_params[k, 0])],
            [np.sin(DH_params[k, 0]), np.cos(DH_params[k, 1]) * np.cos(DH_params[k, 0]), -np.cos(DH_params[k, 0]) * np.sin(DH_params[k, 1]), DH_params[k, 2] * np.sin(DH_params[k, 0])],
            [0, np.sin(DH_params[k, 1]), np.cos(DH_params[k, 1]), DH_params[k, 3]],
            [0, 0, 0, 1]
        ])
        H_matrix = np.dot(H_matrix, H2_matrix)
    return np.matrix(H_matrix)

# def forward_kinematics(q1, q2, q3, q4, q5, q6):
#     X, Y, Z = [0], [0], [0]
#     for k in range(1, 7):
#         H_matrix = DH_Params_H_Matrix(0, k, q1, q2, q3, q4, q5, q6)
#         X.append(float(H_matrix[0, 3]))
#         Y.append(float(H_matrix[1, 3]))
#         Z.append(float(H_matrix[2, 3]))
#     return X, Y, Z

def forward_kinematics(q1, q2, q3, q4, q5, q6):
    X, Y, Z = [0], [0], [0]
    H_prev = Matrix(np.identity(4))
    for k in range(1, 7):
        H_matrix = DH_Params_H_Matrix(0, k, q1, q2, q3, q4, q5, q6)
        X.append(float(H_matrix[0, 3]))
        Y.append(float(H_matrix[1, 3]))
        Z.append(float(H_matrix[2, 3]))
        H_prev = H_matrix
    
    # Extend to gripper using direction of z-axis of last transformation
    z_axis = H_prev[0:3, 2]
    end_effector_pos = np.array([float(H_prev[0, 3]), float(H_prev[1, 3]), float(H_prev[2, 3])])
    gripper_pos = end_effector_pos + a5 * np.array([float(z_axis[0]), float(z_axis[1]), float(z_axis[2])])

    X.append(gripper_pos[0])
    Y.append(gripper_pos[1])
    Z.append(gripper_pos[2])

    return X, Y, Z
    # def forward_kinematics(q1, q2, q3, q4, q5, q6):
    #     X, Y, Z = [0], [0], [0]
    #     for k in range(1, 7):
    #         H_matrix = DH_Params_H_Matrix(0, k, q1, q2, q3, q4, q5, q6)
    #         xk, yk, zk = float(H_matrix[0, 3]), float(H_matrix[1, 3]), float(H_matrix[2, 3])
    #         print(f"Joint {k}: ({xk:.2f}, {yk:.2f}, {zk:.2f})")
    #         X.append(xk)
    #         Y.append(yk)
    #         Z.append(zk)
    #     return X, Y, Z



def update(val):
    x = slider_x.val
    y = slider_y.val
    z = slider_z.val
    roll = slider_roll.val
    pitch = slider_pitch.val
    yaw = slider_yaw.val
    q1, q2, q3, q4, q5, q6 = get_Joint_Variables(x, y, z, roll, pitch, yaw)
    X, Y, Z = forward_kinematics(q1, q2, q3, q4, q5, q6)
    ax.cla()
    ax.plot(X, Y, Z, marker='o', linestyle='-', color='blue')
    ax.set_xlim([-30, 30])
    ax.set_ylim([-30, 30])
    ax.set_zlim([0, 40])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.canvas.draw_idle()

def cosinelaw(a, b, c):
    x = (a * a + b * b - c * c) / (2 * a * b)
    return np.arccos(np.clip(x, -1, 1))

def Gripper_Coordinates(finalX, finalY, finalZ, R0_6):
    Xgripper = finalX - a5 * R0_6[0, 2]
    Ygripper = finalY - a5 * R0_6[1, 2]
    Zgripper = finalZ - a5 * R0_6[2, 2]
    return Xgripper, Ygripper, Zgripper

def inverse_kinematics_first_3(Xgripper, Ygripper, Zgripper):
    q1 = np.arctan2(Ygripper, Xgripper)
    r1 = np.sqrt(Xgripper ** 2 + Ygripper ** 2)
    r2 = np.sqrt((r1 - a2) ** 2 + (Zgripper - a1) ** 2)
    phi1 = np.arctan((Zgripper - a1) / (r1 - a2))
    phi2 = cosinelaw(a3, r2, a4)
    q2 = phi1 + phi2
    phi3 = cosinelaw(a3, a4, r2)
    q3 = phi3 - np.pi
    return q1, q2, q3

def inverse_kinematics_last_3(R3_6):
    q5 = np.arccos(R3_6[2, 2])
    q6 = np.arcsin(R3_6[2, 1] / np.sin(q5))
    q4 = np.arcsin(R3_6[1, 2] / np.sin(q5))
    return q4, q5, q6

def get_Joint_Variables(finalX, finalY, finalZ, roll, pitch, yaw):
    R0_6_initial = np.matrix([
        [0, 0, 1],
        [0, -1, 0],
        [1, 0, 0]
    ])
    R_6_rotation = np.matrix([
        [np.cos(pitch) * np.cos(roll), np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(pitch) * np.cos(yaw), np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw) * np.cos(pitch)],
        [np.sin(roll), np.cos(roll) * np.cos(yaw), -np.sin(yaw) * np.cos(roll)],
        [-np.sin(pitch) * np.cos(roll), np.sin(pitch) * np.sin(roll) * np.cos(yaw) + np.sin(yaw) * np.cos(pitch), -np.sin(pitch) * np.sin(roll) * np.sin(yaw) + np.cos(pitch) * np.cos(yaw)]
    ])
    R0_6 = np.dot(R0_6_initial, R_6_rotation)
    XGripper, YGripper, ZGripper = Gripper_Coordinates(finalX, finalY, finalZ, R0_6)
    q1, q2, q3 = inverse_kinematics_first_3(XGripper, YGripper, ZGripper)
    H0_3 = DH_Params_H_Matrix(0, 3, q1, q2, q3)
    R0_3 = H0_3[0:3, 0:3]
    Inverse_R0_3 = np.linalg.inv(np.matrix(R0_3, dtype=np.float64))
    R3_6 = np.dot(Inverse_R0_3, R0_6)
    q4, q5, q6 = inverse_kinematics_last_3(R3_6)
    return q1, q2, q3, q4, q5, q6

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.25, bottom=0.35)

ax_slider_x = plt.axes([0.25, 0.25, 0.65, 0.03])
ax_slider_y = plt.axes([0.25, 0.2, 0.65, 0.03])
ax_slider_z = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_slider_roll = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_slider_pitch = plt.axes([0.25, 0.05, 0.65, 0.03])
ax_slider_yaw = plt.axes([0.25, 0.0, 0.65, 0.03])

slider_x = Slider(ax_slider_x, 'X', -20.0, 20.0, valinit=0)
slider_y = Slider(ax_slider_y, 'Y', -20.0, 20.0, valinit=0)
slider_z = Slider(ax_slider_z, 'Z', 0.0, 40.0, valinit=20)
slider_roll = Slider(ax_slider_roll, 'Roll', -np.pi, np.pi, valinit=0)
slider_pitch = Slider(ax_slider_pitch, 'Pitch', -np.pi, np.pi, valinit=0)
slider_yaw = Slider(ax_slider_yaw, 'Yaw', -np.pi, np.pi, valinit=0)

slider_x.on_changed(update)
slider_y.on_changed(update)
slider_z.on_changed(update)
slider_roll.on_changed(update)
slider_pitch.on_changed(update)
slider_yaw.on_changed(update)

update(None)
plt.show()
