import logging
import numpy as np
from sympy import Symbol
from scipy.optimize import newton
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from sympy import lambdify
import sympy

logging.basicConfig(filename="log_paths.txt",
                            filemode='a',
                            format='%(message)s',
                            datefmt='%H:%M:%S',
                            level=logging.INFO)

# def cycloid(x0, y0, x_t, y_t, N = 10):
#     def f(theta):
#         return y_t / x_t - (1 - np.cos(theta)) / (theta - np.sin(theta))
#     # circle_theta = newton(f,np.pi/2,tol=1e-13,maxiter=1000)
#     circle_theta = fsolve(f,np.pi/2)
#
#     R = y_t / (1 - np.cos(circle_theta))
#
#     theta = np.linspace(0, circle_theta, N)
#     x = R * (theta - np.sin(theta)) + x0
#     y = R * (1 - np.cos(theta)) + y0
#
#     T = circle_theta * np.sqrt(R)
#     x-=5
#     y-=3
#     return x, y, T, theta

def cycloid(x0, y0, x_t, y_t, N = 3):
    def f(theta):
        return y_t / x_t - (1 - np.cos(theta)) / (theta - np.sin(theta))
    circle_theta = newton(f,np.pi/2)
    R = y_t / (1 - np.cos(circle_theta))
    angle = Symbol('angle')
    eq_x = lambdify(angle,R * (angle - sympy.sin(angle)) - x0)
    eq_y = lambdify(angle,R * (1 - sympy.cos(angle)) - y0)
    solution_x = fsolve(eq_x,np.pi/2)
    solution_y = fsolve(eq_y,np.pi/2)
    theta_x = np.linspace(solution_x, circle_theta, N)
    theta_y = np.linspace(solution_y, circle_theta, N)
    x = R * (theta_x - np.sin(theta_x))
    y = R * (1 - np.cos(theta_y))
    T = circle_theta * np.sqrt(R)
    x-=5
    y-=3
    return x,y,T