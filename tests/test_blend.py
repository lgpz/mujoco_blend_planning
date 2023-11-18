import numpy as np
from spatialmath import SO3
import matplotlib.pyplot as plt

from src.motion_planning import BlendPlanner, LinePositionParameter, OneAttitudeParameter, CartesianParameter

if __name__ == '__main__':
    t0 = np.array([0.0, 0.0, 0.0])
    R0 = SO3()

    t1 = np.array([1.0, 1.0, 1.0])
    R1 = SO3()

    t2 = np.array([2.0, 0.0, 0.5])
    R2 = SO3()

    t3 = np.array([3.0, 1.0, 1.0])
    R3 = SO3()

    t4 = np.array([4.0, 0.0, 0.5])
    R4 = SO3()

    line_attitude_parameter0 = LinePositionParameter(t0, t1)
    one_attitude_parameter0 = OneAttitudeParameter(R0, R1)
    cartesian_parameter0 = CartesianParameter(line_attitude_parameter0, one_attitude_parameter0)

    line_attitude_parameter1 = LinePositionParameter(t1, t2)
    one_attitude_parameter1 = OneAttitudeParameter(R1, R2)
    cartesian_parameter1 = CartesianParameter(line_attitude_parameter1, one_attitude_parameter1)

    line_attitude_parameter2 = LinePositionParameter(t2, t3)
    one_attitude_parameter2 = OneAttitudeParameter(R2, R3)
    cartesian_parameter2 = CartesianParameter(line_attitude_parameter2, one_attitude_parameter2)

    line_attitude_parameter3 = LinePositionParameter(t3, t4)
    one_attitude_parameter3 = OneAttitudeParameter(R3, R4)
    cartesian_parameter3 = CartesianParameter(line_attitude_parameter3, one_attitude_parameter3)

    cartesian_parameters = [cartesian_parameter0, cartesian_parameter1, cartesian_parameter2, cartesian_parameter3]
    radii = [0.5 for _ in range(len(cartesian_parameters) - 1)]

    blend_planner = BlendPlanner(cartesian_parameters, radii)

    px = []
    py = []
    pz = []

    s = np.linspace(0.0, 1.0, 1001)
    for si in s:
        interpolate = blend_planner.interpolate(si)
        px.append(interpolate.t[0])
        py.append(interpolate.t[1])
        pz.append(interpolate.t[2])

    dx_ds = np.gradient(px, s)
    dy_ds = np.gradient(py, s)
    dz_ds = np.gradient(pz, s)

    d2x_ds2 = np.gradient(dx_ds, s)
    d2y_ds2 = np.gradient(dy_ds, s)
    d2z_ds2 = np.gradient(dz_ds, s)

    plt.figure(1)
    plt.plot(s, px, label='x', color='r', linestyle='-')
    plt.plot(s, py, label='y', color='g', linestyle='-')
    plt.plot(s, pz, label='z', color='b', linestyle='-')
    plt.xlabel('s')
    plt.ylabel('Position')
    plt.legend()

    plt.figure(2)
    plt.plot(s, dx_ds, label='dx/ds', color='r', linestyle='-')
    plt.plot(s, dy_ds, label='dy/ds', color='g', linestyle='-')
    plt.plot(s, dz_ds, label='dz/ds', color='b', linestyle='-')
    plt.xlabel('s')
    plt.ylabel('Derivative')
    plt.legend()

    plt.figure(3)
    plt.plot(s, d2x_ds2, label='d2x/ds2', color='r', linestyle='-')
    plt.plot(s, d2y_ds2, label='d2y/ds2', color='g', linestyle='-')
    plt.plot(s, d2z_ds2, label='d2z/ds2', color='b', linestyle='-')
    plt.xlabel('s')
    plt.ylabel('Second Derivative')
    plt.legend()

    plt.figure(4)
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(px, py, pz)
    ax1.set_xlabel(r'$x$')
    ax1.set_ylabel(r'$y$')
    ax1.set_zlabel(r'$z$')
    plt.show()
