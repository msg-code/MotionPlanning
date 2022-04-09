from operator import imod
from pyparsing import alphas
from sympy import *
from sympy.abc import T


p_xf = symbols('p_xf')
p_yf = symbols('p_yf')
p_zf = symbols('p_zf')
p_x0 = symbols('p_x0')
p_y0 = symbols('p_y0')
p_z0 = symbols('p_z0')
v_x0 = symbols('v_x0')
v_y0 = symbols('v_y0')
v_z0 = symbols('v_z0')
## 默认采用delta_v = 0
delta_p_x = p_xf - v_x0 * T - p_x0
delta_p_y = p_yf - v_y0 * T - p_y0
delta_p_z = p_zf - v_z0 * T - p_z0


alpha_1 = -12 / (T**3) * delta_p_x
alpha_2 = -12 / (T**3) * delta_p_y
alpha_3 = -12 / (T**3) * delta_p_z

# pprint(alpha_1)
# pprint(alpha_2)
# pprint(alpha_3)
beta_1 = 6 / (T**2) * delta_p_x
beta_2 = 6 / (T**2) * delta_p_y
beta_3 = 6 / (T**2) * delta_p_z
# pprint(beta_1)
# pprint(beta_2)
# pprint(beta_3)

J = T + (1 / 3 * (alpha_1**2) * (T**3) + alpha_1 * beta_1 * (T**2) + (beta_1**2) * T) + (1 / 3 * (alpha_2**2) * (T**3) + alpha_2 * beta_2 * (T**2) + (beta_2**2) * T) + (1 / 3 * (alpha_3**2) * (T**3) + alpha_3 * beta_3 * (T**2) + (beta_3**2) * T)
J = simplify(J)
pprint(J)
J_final = series(J,T)
pprint(J_final)
##
###
den = T**3
num = den * J

derivative = Poly(
    collect(
        expand(
            diff(num, T)*den - num*diff(den, T)
        ),
        syms=T
    ),
    T
)
#
# done:
#
for order, coeff in zip(
    range(len(derivative.all_coeffs()) - 1, -1, -1),
    derivative.all_coeffs()
):
    print(f"{order}: {coeff}\n")


