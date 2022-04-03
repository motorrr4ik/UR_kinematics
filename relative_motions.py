import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt
from forward_inverse_kinematics import forward_kinematics_solution
from forward_inverse_kinematics import inverse_kinematics
from forward_inverse_kinematics import jacobian_solution, manipulability_metrics, coop_jacobian
from forward_inverse_kinematics import Rx, Ry, Rz

config_matrix = np.array([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

# P0 = np.array([[0],[0],[0.5]])
Z = 0.262
x = np.arange(-1, 1.05, 0.05)
y = np.arange(-1, 1.05, 0.05)

# P0 = np.array(10*np.random.rand(9,9))
# P0[0, :] = 1
# P0[4,:] = 1
# P0[8, :] = 1
# print(P0)
P_r = np.array([0, 0, 0.05])
# w = np.array(np.zeros((9,3)))
w = np.array(np.zeros((len(x),len(y))))
# w2 = np.array(np.zeros((1,6)))

R0 = Rx(pi/2)
R_r = Rx(pi)
R_between = np.eye(3)
# print(R0)
# print()
# print(R_r)
# print()

def p_vecs(P0, P_r):
    P_vec = np.array([P0, P_r])
    A = np.array(np.zeros((6,6)))
    A[:3, :3] = 0.5*np.eye(3)
    A[:3, 3:] = 0.5*np.eye(3)
    A[3:, :3] = 1*np.eye(3)
    A[3:, 3:] = -1*np.eye(3)
    A_inv = np.linalg.inv(A)
    P_vec = P_vec.flatten()
    P_vec = A_inv.dot(P_vec)
    # print(P_vec[:3])
    # print(P_vec[3:])
    return P_vec

def calc_rot(R_12, R2, R_r):
    # R1 = np.linalg.inv(R_r).dot(R2).dot(R_12)
    R1 = (R_r.T).dot(R2).dot(R_12)
    # R1 = R_r.dot(np.linalg.inv(R2)).dot(np.linalg.inv(R_12))
    # print(R)
    return R1

def calc_T_mat(R, p):
    T_mat = np.array(np.zeros((4,4))) 
    T_mat[:3,:3] = R
    T_mat[:3, 3] = p
    T_mat[3,3] = 1
    # print(T_mat)
    return T_mat


R1 = calc_rot(R_between, R0, R_r)
R2 = R0
# print(R1)
# print(R2)



for i in range(len(x)):
    for j in range(len(y)):
        # print(P0[3*i:3*(i+1),j])
        P0 = np.array([x[i], y[j], Z])
        p_vectors = p_vecs(P0, P_r)
        # print("P vectors:")
        # print(p_vectors.flatten())

        T1 = calc_T_mat(R1.T, p_vectors[:3])
        # print("T1:")
        # print(T1)
        T2 = calc_T_mat(R2, p_vectors[3:])
        # print("T2:")
        # print(T2)

        # print(T1)
        try:
            theta1 = inverse_kinematics(config_matrix, T1)
            theta2 = inverse_kinematics(config_matrix, T2)
        except:
            continue
        # print("theta 1: ", theta1[:,:1])
        # print("theta 2: ", theta2[:,:2])
        
        J1 = jacobian_solution(config_matrix, 6, theta1[:, 0].flatten())
        J2 = jacobian_solution(config_matrix, 6, theta2[:, 0].flatten())
        Jc = coop_jacobian(J1, J2)

        # print(J1)
        # print(J2)

        w[i,j] = manipulability_metrics(Jc[6:, :])
        
        # w[j,i] = manipulability_metrics(J1)
    
        # w2 = np.append(w2, manipulability_metrics(J2))
        # print(w)

# print(w[0,0])

figure, ax = plt.subplots()
im = ax.imshow(w, interpolation = 'nearest')

ax.set_xticks(np.arange(len(x)))
ax.set_yticks(np.arange(len(y)))
ax.set_xticklabels(np.round(x,2))
ax.set_yticklabels(np.round(y,2))
plt.setp(ax.get_xticklabels(), rotation=90, ha="right",
         rotation_mode="anchor")

# for i in range(w.shape[0]):
#     for j in range(w.shape[1]):
#         ax.text(j, i, np.round(w[i,j],1), ha="center", va="center", color="w")

ax.set_aspect('equal')
ax.set_title("Heatmap")
# divider = make_axes_locatable(ax)
# cax = divider.append_axes("right", "3%", pad="1%")
ax.figure.colorbar(im, ax=ax)
figure.tight_layout()
plt.show()
    


