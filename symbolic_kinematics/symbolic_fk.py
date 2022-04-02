from . import Tz_d, Tz_theta, Tx_a, Tx_alpha
import sympy as sym

def symbolic_transformation_matrix(i):
    T = Tz_theta(i)*Tz_d(i)*Tx_a(i)*Tx_alpha(i)
    # print(T)
    T = sym.Matrix(T).applyfunc(sym.simplify)
    # print(T)
    return T

def symbolic_fk_solution(joints):
    T_final = sym.eye(4)
    for i in range(1, joints+1):
        T_current = symbolic_transformation_matrix(i)
        T_final = T_final*T_current
        T_final = sym.simplify(T_final)
        # T_final = sym.expand(sym.Matrix(T_final))
        # T_final = sym.simplify(sym.Matrix(T_final))
        # T_final = sym.expand_trig(sym.Matrix(T_final))
        # T_final = sym.trigsimp(sym.Matrix(T_final))
    return T_final