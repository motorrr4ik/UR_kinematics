import sympy as sym
import numpy as np
from symbolic_kinematics import symbolic_manipulability_metrics, symbolic_fk_solution, SYMBOLIC_CONFIG_MATRIX, symbolic_jacobian_solution
JOINTS_NUMBER = 6

T = symbolic_fk_solution(6)
# J = symbolic_jacobian_solution(6)
# w = symbolic_manipulability_metrics(J)
print(T)