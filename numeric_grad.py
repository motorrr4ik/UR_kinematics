from urllib.robotparser import RobotFileParser
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, fmin_cg, fmin_bfgs, fmin_tnc, golden, fsolve, minimize
from forward_inverse_kinematics import inverse_kinematics, jacobian_solution, coop_jacobian, transformation_matrix, forward_kinematics_solution, Rx, Ry, Rz, manipulability_metrics

config_matrix = np.array([[0, np.pi/2.0, 0.1625],
                          [-0.425, 0, 0],
                          [-0.3922, 0, 0],
                          [0, np.pi/2.0, 0.1333],
                          [0, -np.pi/2.0, 0.0997],
                          [0, 0, 0.0996]])

P_r = np.array([0, 0, 0.05])
# P_ak = np.array([0.23, 0.33, 0.43])
P_ak = np.array([0.1, 0.1, 0.1])
P_akp = np.array([0.5, 0.55, 0.75])

h = 0.5
k = 1
#################################################
A = np.array(np.zeros((6,6)))
A[:3, :3] = 0.5*np.eye(3)
A[:3, 3:] = 0.5*np.eye(3)
A[3:, :3] = 1*np.eye(3)
A[3:, 3:] = -1*np.eye(3)
#################################################
def get_P_vecs(P_a, P_r):
    A_inv = np.linalg.inv(A)
    P_vecs = np.array([P_a, P_r]).flatten()
    P12 = A_inv.dot(P_vecs)
    return P12
##################################################
def get_T(R, P):
    T = np.array(np.zeros((4,4)))
    T[:3, :3] = R
    T[:3, 3] = P
    T[3,3] = 1
    return T
##################################################
def get_R(R_12, R2, R_r):
    # R1 = np.linalg.inv(R_r).dot(R2).dot(R_12)
    R1 = (R_r.T).dot(R2).dot(R_12)
    # R1 = R_r.dot(np.linalg.inv(R2)).dot(np.linalg.inv(R_12))
    # print(R)
    return R1
##################################################
def get_metrics(T1, T2):
    IK1 = inverse_kinematics(config_matrix, T1)
    IK2 = inverse_kinematics(config_matrix, T2)
    q1 = IK1[:, 0].flatten()
    q2 = IK2[:, 0].flatten()
    J1 = jacobian_solution(config_matrix, 6, q1)
    J2 = jacobian_solution(config_matrix, 6, q2)
    J_coop = coop_jacobian(J1,J2)
    w = manipulability_metrics(J_coop)
    return w
##################################################
def from_Pa_to_mu(P_a, R1, R2):
    P12 = get_P_vecs(P_a, P_r)
    T1 = get_T(np.transpose(R1), P12[:3])
    T2 = get_T(R2, P12[3:])
    w = get_metrics(T1, T2)
    return w

R0 = Rx(np.pi/2)
R_r = Rx(np.pi)
R_between = np.eye(3)
R2 = R0
R1 = get_R(R_between, R2, R_r)

test = np.array([0.1, -0.75, 0.5])
test2 = np.array([-0.75, 0.1, 0.5])
w = from_Pa_to_mu(test, R1, R2)
w2 = from_Pa_to_mu(test2, R1, R2)
print(w)
print(w2)

# w_max = fmin(from_Pa_to_mu, P_ak, args=(R1, R2))
# print(w_max)

# for i in range(0,100):

#     Px = np.array([P_ak[0], P_akp[1], P_akp[2]])
#     Py = np.array([P_akp[0], P_ak[1], P_akp[2]])
#     Pz = np.array([P_akp[0], P_akp[1], P_ak[2]])
#     # print(Px, Py, Pz)
#     # Px = np.array([P_ak[0]+h, P_akp[1], P_akp[2]])
#     # Py = np.array([P_akp[0], P_ak[1]+h, P_akp[2]])
#     # Pz = np.array([P_akp[0], P_akp[1], P_ak[2]+h])
#     wx = from_Pa_to_mu(Px)
#     wy = from_Pa_to_mu(Py)
#     wz = from_Pa_to_mu(Pz)
#     w_p = from_Pa_to_mu(P_akp)
#     # print(wx, wy, wz)
#     # print(k*(wx-w_p)/h, k*(wy-w_p)/h, k*(wz-w_p)/h)

#     # P_aknx = P_ak[0] + k*(wx-w_p)/h
#     # P_akny = P_ak[1] + k*(wy-w_p)/h
#     # P_aknz = P_ak[2] + k*(wz-w_p)/h

#     P_aknx = P_ak[0] + wx/(k*(wx-w_p)/h)
#     P_akny = P_ak[1] + wy/(k*(wy-w_p)/h)
#     P_aknz = P_ak[2] + wz/(k*(wz-w_p)/h)

#     # P_aknx = (P_ak[0]+(i/1000)) - k*(w-w_p)/h
#     # P_akny = (P_ak[1]+(i/1000)) - k*(w-w_p)/h
#     # P_aknz = (P_ak[2]+(i/1000)) - k*(w-w_p)/h
#     P_akn = np.array([P_aknx, P_akny, P_aknz])
#     # P_akn = P_aknx + P_akny + P_aknz
#     print(P_akn)
#     P_akp  = P_ak
#     P_ak = P_akn

# print(P_ak)







