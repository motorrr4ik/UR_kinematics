import numpy as np
from numpy import linalg

def self_inverse_kinematics(config_matrix, transformation_matrix):
    theta = np.zeros((6,8))
    T = transformation_matrix

    P_05 = T.dot(np.array([[0],[0], [-config_matrix[5,2]], [1]]))
    
    # P_06 = T[:,3]
    alpha = np.arctan2(P_05[1], P_05[0])
    beta = np.arcsin(config_matrix[3,2]/np.sqrt(P_05[0]**2 + P_05[1]**2))
    # print(np.sqrt(P_05[0]**2 + P_05[1]**2))
    # print(alpha)
    # print(beta)
    theta[0, 0:4] = alpha + beta
    theta[0, 4:8] = alpha - beta + np.pi
    return theta
