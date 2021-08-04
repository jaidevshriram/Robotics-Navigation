import argparse

import cv2
import numpy as np
import cvxpy as cp
import magnum as mn
import dccp
import matplotlib.pyplot as plt

from habitat.utils.visualizations import maps
from habitat_sim.utils import common as utils
from PIL import Image

from sim import make_sim
from map import get_map, make_blobs
from load_obstacles import detect_obstacles
from config import N_HORIZON, MAX_VELOCITY, MIN_VELOCITY, MAX_ACCELERATION, MIN_ACCELERATION, BUFFER

# parser = argparse.ArgumentParser()
# parser.add_argument("--map", default="./maps/obstacle.png")
# parser.add_argument("--start", nargs='+', type=float, required=True)
# parser.add_argument("--end", nargs='+', type=float, required=True)

def mpc(sim, map, obstacles, start, end):

    start = start.reshape(-1, 1)
    end = end.reshape(-1, 1)

    x = [cp.Variable((2, 1)) for i in range(N_HORIZON + 1)] # Position matrix - 2 x N + 1 
    dx = [cp.Variable((2, 1)) for i in range(N_HORIZON + 1)] # Velocity matrix - 2 X N + (1) : (because acceleration is pushing it!)
    a = [cp.Variable((2, 1)) for i in range(N_HORIZON)]
    
    A = np.eye(2) # Delta T^2 matrix
    B = np.eye(2) # Convenience for multiplication

    max_dx = np.array(MAX_VELOCITY, dtype=np.float64).reshape(-1, 1)
    min_dx = np.array(MIN_VELOCITY, dtype=np.float64).reshape(-1, 1)

    max_a = np.array(MAX_ACCELERATION, dtype=np.float64).reshape(-1, 1)
    min_a = np.array(MIN_ACCELERATION, dtype=np.float64).reshape(-1, 1)

    objective = 0
    constraints = 0

    # Starting Condition
    x_init = cp.Parameter((2, 1))
    constraints = [x[0] == x_init]

    for i in range(1, N_HORIZON + 1):
        objective += cp.quad_form(x[i] - end, A)
        constraints += [x[i] == x[i-1] + B @ dx[i-1]]
        constraints += [dx[i-1] <= max_dx]
        constraints += [dx[i-1] >= min_dx]

        constraints += [dx[i] == dx[i - 1] + B @ a[i - 1]]
        constraints += [a[i-1] <= max_a]
        constraints += [a[i-1] >= min_a]

        for j in obstacles:
            obs_pos = np.array(j[0:2]).reshape(-1, 1)
            constraints += [cp.norm(x[i] - obs_pos) >= (j[2] + BUFFER)]

    problem = cp.Problem(cp.Minimize(objective), constraints)

    # Solve the problem at every step
    nsim = 15

    out = cv2.VideoWriter('rgb_trajectory.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 3.0, (512, 512))

    xs = []
    ys = []

    # Solve the problem at every step
    while True:
        print("Current loc:", start.flatten())
        x_init.value = start
        problem.solve(method='dccp')
        print("--Moving by ", dx[0].value.flatten(), "/s")
        start = x[1].value

        if np.allclose(start, end) == True:
            break

        xs.append(start.flatten()[0])
        ys.append(start.flatten()[1])

        agent = sim.get_agent(0)
        current_state = agent.get_state()
        _x, _y = position = maps.from_grid(
            start.flatten()[1],
            start.flatten()[0],
            (map.shape[0], map.shape[1]),
            pathfinder=sim.pathfinder
        )

        x_new, y_new = maps.from_grid(
            x[2].value.flatten()[1],
            x[2].value.flatten()[0],
            (map.shape[0], map.shape[1]),
            pathfinder=sim.pathfinder
        )

        current_state.position = mn.Vector3([_y, 0, _x])
        current_state.rotation = utils.quat_from_magnum(mn.Quaternion.from_matrix(
            mn.Matrix4.look_at(
                mn.Vector3([_y, 0, _x]), 
                mn.Vector3([y_new, 0, x_new]),
                np.array([0, 1.0, 0])
            ).rotation()
        ))
        agent.set_state(current_state)

        observations = sim.get_sensor_observations()
        rgb = observations['rgb']
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
        
        # map = plt.gca().get_array()

        # img = cv2.hnconcat([rgb, map])
        # print(img.shape)

        out.write(rgb)

    plt.imshow(map, cmap='Greys')
    plt.plot(xs, ys, marker='X')
    plt.show()
    out.release()

    return start

if __name__ == "__main__":
    # args = parser.parse_args()

    # start = tuple(args.start)
    # end = tuple(args.end)
    start = np.array([250, 380], dtype=np.float64)
    end = np.array([110, 160], dtype=np.float64)

    sim = make_sim("/mnt/c/Users/jaide/Desktop/Robotics/Habitat-Frontier-Exploration/data")
    map = get_map(sim)

    obstacle_list = detect_obstacles(map)
    # print(obstacle_list)
    # obstacle_list = [(183.919189453125, 262.0182800292969, 54.65785217285156)]
    ret = mpc(sim, map, obstacle_list, start, end)
