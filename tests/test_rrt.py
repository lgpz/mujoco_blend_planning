import numpy as np
import matplotlib.pyplot as plt

from spatialmath import SO3
from src.motion_planning import RRTPlanner, RRTParameter, RRTMap, BlendPlanner

if __name__ == '__main__':
    show_animation = True

    obstacles = [
        (4, 10, 3.0),
        (8.0, 5.0, 2.0),
        (12.0, 11.0, 2.0),
        (17, 13, 2.0),
    ]

    rrt_map = RRTMap(area=[(-2.0, -2.0), (20.0, 20.0)], obstacles=obstacles)
    rrt_parameter = RRTParameter(start=[0.0, 0.0], goal=[18.0, 18.0], expand_dis=2.0, max_iter=200,
                                 animation=show_animation)
    rrt_planner = RRTPlanner(rrt_map, rrt_parameter)

    path_parameters = rrt_planner.get_path_parameters(SO3.RPY(-np.pi, 0.0, -np.pi / 2))

    radii = [1.0 for _ in range(len(path_parameters) - 1)]

    blend_planner = BlendPlanner(path_parameters, radii)

    px_raw = []
    py_raw = []
    pz_raw = []

    px_blend = []
    py_blend = []
    pz_blend = []

    s = np.linspace(0.0, 1.0, 1001)
    for si in s:
        interpolate_raw = rrt_planner.interpolate(si)
        px_raw.append(interpolate_raw.t[0])
        py_raw.append(interpolate_raw.t[1])
        pz_raw.append(interpolate_raw.t[2])

        interpolate_blend = blend_planner.interpolate(si)
        px_blend.append(interpolate_blend.t[0])
        py_blend.append(interpolate_blend.t[1])
        pz_blend.append(interpolate_blend.t[2])

    plt.figure(1)
    plt.plot(s, px_raw, label='x_raw', color='r', linestyle='-')
    plt.plot(s, py_raw, label='y_raw', color='g', linestyle='-')
    plt.plot(s, pz_raw, label='z_raw', color='b', linestyle='-')
    plt.plot(s, px_blend, label='x_blend', color='r', linestyle='--')
    plt.plot(s, py_blend, label='y_blend', color='g', linestyle='--')
    plt.plot(s, pz_blend, label='z_blend', color='b', linestyle='--')
    plt.xlabel('s')
    plt.ylabel('Position')
    plt.legend()

    plt.figure(2)
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(px_raw, py_raw, pz_raw, label='raw', linestyle='-')
    ax1.plot3D(px_blend, py_blend, pz_blend, label='blend_planning', linestyle='--')
    ax1.set_xlabel(r'$x$')
    ax1.set_ylabel(r'$y$')
    ax1.set_zlabel(r'$z$')
    plt.legend()

    plt.show()
