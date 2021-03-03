import logging
import numpy as np
from sympy import Symbol
from scipy.optimize import newton
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from sympy import lambdify
from scipy.integrate import quad
import sympy

def func(x, f, fp):
    """The integrand of the time integral to be minimized for a path f(x)."""

    return np.sqrt((1+fp(x)**2) / (2 * f(x)))

def parabola(x0, x2, y2, N=100):
    """Return the path of a parabolic arc between (0,0) to (x2, y2).

    The parabola used is the one with a vertical tangent at (0,0).

    """

    c = (y2/x2)**2

    def f(x):
        return np.sqrt(c*x)
    def fp(x):
        return c/2/f(x)

    x = np.linspace(x0, x2, N)
    y = f(x)

    # Calcualte the time of travel by numerical integration.
    T = quad(func, 0, x2, args=(f, fp))[0]
    print('T(parabola) = {:.3f}'.format(T))
    return x, y, T

def linear(x0,x2, y2, N=100):
    """Return the path of a straight line from (0,0) to (x2, y2)."""

    m = y2 / x2
    x = np.linspace(x0, x2, N)
    y = m*x

    # The time of travel
    T = np.sqrt(2*(1+m**2)/m * x2)
    print('T(linear) = {:.3f}'.format(T))
    return x, y, T

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