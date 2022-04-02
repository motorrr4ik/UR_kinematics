from cmath import cos
import sympy as sym
SYMBOLIC_CONFIG_MATRIX = sym.Matrix([[0, sym.Symbol('alpha1'), sym.Symbol('d1')],
                                    [sym.Symbol('-a2'), 0, 0],
                                    [sym.Symbol('-a3'), 0, 0],
                                    [0, sym.Symbol('alpha4'), sym.Symbol('d4')],
                                    [0, sym.Symbol('alpha5'), sym.Symbol('d5')],
                                    [0, 0, sym.Symbol('d6')]])

def Rx(i):
    rx = sym.Matrix([[1, 0, 0],
                [0, sym.cos(sym.Symbol('theta_%d'%(i))), -sym.sin(sym.Symbol('theta_%d'%(i)))],
                [0, sym.sin(sym.Symbol('theta_%d'%(i))), sym.cos(sym.Symbol('theta_%d'%(i)))]]) 
    return rx

def Ry(i):
    ry = sym.Matrix([[sym.cos(sym.Symbol('theta_%d'%(i))), 0, sym.sin(sym.Symbol('theta_%d'%(i)))],
                [0, 1, 0],
                [-sym.sin(sym.Symbol('theta_%d'%(i))), 0, sym.cos(sym.Symbol('theta_%d'%(i)))]]) 
    return ry

def Rz(i):
    rz = sym.Matrix([[sym.cos(sym.Symbol('theta_%d'%(i))), -sym.sin(sym.Symbol('theta_%d'%(i))), 0],
                [sym.sin(sym.Symbol('theta_%d'%(i))), sym.cos(sym.Symbol('theta_%d'%(i))), 0],
                [0, 0, 1]]) 
    return rz

def Tz_theta(i):
    i = i - 1
    T = sym.Matrix([[sym.cos(sym.Symbol('theta_%d'%(i))), -sym.sin(sym.Symbol('theta_%d'%(i))), 0, 0],
                        [sym.sin(sym.Symbol('theta_%d'%(i))), sym.cos(sym.Symbol('theta_%d'%(i))), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    return T

def Tz_d(i):
    i = i - 1
    T = sym.eye(4)
    T[2,3] = SYMBOLIC_CONFIG_MATRIX[i,2]
    return T

def Tx_a(i):
    i = i - 1
    T = sym.eye(4)
    T[0,3] = SYMBOLIC_CONFIG_MATRIX[i,0]
    return T

def Tx_alpha(i):
    i = i - 1
    T  = sym.Matrix([[1, 0, 0, 0],
                [0, sym.cos(SYMBOLIC_CONFIG_MATRIX[i,1]), -sym.sin(SYMBOLIC_CONFIG_MATRIX[i,1]), 0],
                [0, sym.sin(SYMBOLIC_CONFIG_MATRIX[i,1]), sym.cos(SYMBOLIC_CONFIG_MATRIX[i,1]), 0],
                [0, 0, 0, 1]])
    return T