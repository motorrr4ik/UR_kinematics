import sympy as sym
import numpy as np
from . import SYMBOLIC_CONFIG_MATRIX, symbolic_fk_solution, symbolic_transformation_matrix

def symbolic_jacobian_solution(joints):
    J = sym.zeros(6,joints)
    P6 = symbolic_fk_solution(joints)[:3,3]
    # print(P6)
    # P6 = sym.Matrix(sym.simplify(P6))
    Z0 = sym.Matrix([0,0,1])
    P0 = sym.Matrix([0,0,0])
    # J[:3,0] = np.cross(cur_z.flatten(),(p6 - cur_p.flatten()))
    # J[:3,0] = sym.flatten(Z0).cross(sym.flatten(P6-P0))
    # J[3:,0] = sym.flatten(Z0)
    J[:3,0] = Z0.cross(P6-P0)
    J[3:,0] = Z0

    for i in range(1, joints):
        T_i = symbolic_fk_solution(i)
        P_i = T_i[:3,3]
        Z_i = T_i[:3, 2]
        J[:3, i] = sym.Matrix(Z_i).cross(P6-P_i)
        J[3:, i] = Z_i
    J = sym.expand(J)
    # print(J)
    return J

def coop_symbolic_jacobian(J_left, J_right):
    coop_J = sym.Matrix(sym.zeros((12,12)))
    coop_J[:6, :6] = 0.5*J_left
    coop_J[:6, 6:] = 0.5*J_right
    coop_J[6:,:6] = J_left
    coop_J[6:,6:] = J_right 

    return coop_J

def symbolic_manipulability_metrics(J):
    multi_J = sym.Matrix(J)*sym.Matrix(J).T
    multi_J = sym.expand_trig(multi_J)
    det = sym.Matrix(multi_J).det()
    w = sym.sqrt(det)
    print(multi_J)
    return w