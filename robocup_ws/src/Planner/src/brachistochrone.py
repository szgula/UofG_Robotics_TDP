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

def cycloid(x0, y0, x_t, y_t, V, N = 6):
    wheel_radius = 0.02
    def f(theta):
        return y_t / x_t - (1 - np.cos(theta)) / (theta - np.sin(theta))
    circle_theta = newton(f,np.pi/2,maxiter=1000)
    # circle_theta = newton(f,np.pi/2)
    # circle_theta = fsolve(f,np.pi/2)
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
    T = circle_theta * np.sqrt(R / g)
    Vx = V*np.cos(theta_x)
    Vy = V*np.sin(theta_y)
    w = np.sqrt((Vx**2 + Vy**2)/R)
    Vr = w*(R + wheel_radius/2)
    Vl = w*(R - wheel_radius/2)
    return x, y, T, Vr, Vl

g = 9.81
trajectory = np.array([[1.5, 1.2],
                       [2.5, 3],
                       [4, 7.5],
                       [10, 8]])
fig, ax = plt.subplots()
path = []
time = []
init_x = 0.5
init_y = 0.5
V = 2
logging.info("#######################")
for point in trajectory:
    x, y, T, Vr, Vl = cycloid(init_x,
                      init_y,
                      point[0],
                      point[1], V)
    path.append([x,y])
    time.append(T)
    logging.info(f"Right Wheel: {np.mean(Vr)}\nLeft Wheel: {np.mean(Vl)}")
    logging.info(f"({init_x}, {init_y}) => ({point[0]}, {point[1]})")
    logging.info(f"x: {x}")
    logging.info(f"y: {y}")
    logging.info(f"Time taken: {T} seconds")
    init_x += 0.5
    init_y += 0.5
