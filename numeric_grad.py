import numpy as np
from forward_inverse_kinematics import inverse_kinematics, jacobian_solution, coop_jacobian, transformation_matrix, forward_kinematics_solution, Rx, Ry, Rz, manipulability_metrics

config_matrix = np.array([[0, np.pi/2.0, 0.1625],
                          [-0.425, 0, 0],
                          [-0.3922, 0, 0],
                          [0, np.pi/2.0, 0.1333],
                          [0, -np.pi/2.0, 0.0997],
                          [0, 0, 0.0996]])

P_r = np.array([0, 0, 0.05])
P_ak = np.array([0.85, 0.63, 0.23])
P_akp = np.array([0.5, 0.55, 0.75])
R_orientation = Rz(np.pi/2)
h = 0.5
k = 0.3
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
    T[:3, 3] = P[:3]
    T[3,3] = 1
    return T
##################################################
def get_metrics(T1, T2):
    IK1 = inverse_kinematics(config_matrix, T1)
    IK2 = inverse_kinematics(config_matrix, T2)
    q1 = IK1[:, 1].flatten()
    q2 = IK2[:, 1].flatten()
    J1 = jacobian_solution(config_matrix, 6, q1)
    J2 = jacobian_solution(config_matrix, 6, q2)
    J_coop = coop_jacobian(J1,J2)
    w = manipulability_metrics(J_coop)
    return w

P12 = get_P_vecs(P_ak, P_r)
P12_p = get_P_vecs(P_akp, P_r)

T1 = get_T(R_orientation, P12[:3])
T1_p = get_T(R_orientation, P12_p[:3])
T2 = get_T(R_orientation, P12[3:])
T2_p = get_T(R_orientation, P12_p[3:])

w = get_metrics(T1, T2)
w_p = get_metrics(T1_p, T2_p)


P_akn = P_ak - k*(w-w_p)/h
print(P_akn)







