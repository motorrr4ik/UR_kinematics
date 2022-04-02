from math import pi
import numpy as np
from forward_inverse_kinematics import forward_kinematics_solution
from forward_inverse_kinematics import inverse_kinematics
from forward_inverse_kinematics import self_inverse_kinematics
from forward_inverse_kinematics import jacobian_solution

# config_matrix = np.array([[0, -pi/2.0, 0.1625],
#                         [0.425, 0, 0],
#                         [0.3922, 0, 0],
#                         [0, -pi/2.0, 0.1333],
#                         [0, pi/2.0, 0.0997],
#                         [0, 0, 0.0996]])

config_matrix = np.array([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

ed = np.array([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]])
# ed = np.array([-2.81189996, -1.32400677,  1.14284159, -3.20948596, -0.47036623,-0.00401426])
# ed = np.array([[0], [-pi/2], [0], [-pi/2],[0], [0]])
# ed = np.array([[0.841471], [0.841471], [0.841471], [0.841471],
#                     [0.841471], [0.841471]])
# T03 = forward_kinematics_solution(config_matrix, 3, ed)
# T06 = forward_kinematics_solution(config_matrix, 6, ed)

print("Current angles")
print(ed)
transform = forward_kinematics_solution(config_matrix, 6, ed)
# orientation = R.from_matrix(transform[:3,:3]).as_rotvec()
print("Forward")
# print(transform)
print("Inverse")
IKS = inverse_kinematics(config_matrix, transform)
my_iks = self_inverse_kinematics(config_matrix, transform)
print("all")         #all    
print(IKS)
print(my_iks)
# jc = jacobian_solution(config_matrix, 6, ed)
# print(jc)
# T_test = np.array([[0,1,0,0],
#      [1, 0, 0,10.0],
#      [0, 0,-1,  0],
#      [0, 0, 0,  1]])
# test_iks = inverse_kinematics(config_matrix, T_test)
# print(test_iks)
# IKS = inverse_kinematics_second(config_matrix, T03, T06)
# print("all")         #all    
# print(IKS)
