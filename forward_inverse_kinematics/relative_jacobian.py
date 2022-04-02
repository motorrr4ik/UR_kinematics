from forward_kinematics import forward_kinematics_solution, transformation_matrix
import numpy as np 
import matplotlib.pyplot as plt

def relative_jacobian_matrix(left_T_mat, right_T_mat, left_J, right_J, y_position):
    relative_jacobian = np.array(np.zeros((6,6)))

    relative_jacobian[:3,:3] = right_T_mat[:3,:3].T 
    relative_jacobian[3:, 3:] = right_T_mat[:3,:3].T
    left_gamma = __gammArix(left_T_mat)
    right_gamma = __gammArix(right_T_mat)
    gamma_array = np.array([left_gamma.dot(left_J), -right_gamma.dot(right_J)]) 
    
    relative_jacobian = relative_jacobian.dot(gamma_array[0]) - relative_jacobian.dot(gamma_array[1])



    return relative_jacobian

def __gammArix(T_mat):
    gammA = np.array(np.zeros((6,6)))
    gammA[:3, :3] = np.array(np.eye(3))
    gammA[3:, 3:] = np.array(np.eye(3))
    gammA[3, 3] = y_position -  T_mat[:3, 3]
    return gammA


# relative_jacobian_matrix(0, 0, 0, 0)