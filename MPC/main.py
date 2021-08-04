import argparse
import cv2
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import dccp
from time import time
import warnings
warnings.filterwarnings("ignore")

from simulate import simulate
from helpers import load_map
from load_obstacles import detect_obstacles
from config import N_HORIZON, MAX_VELOCITY, MIN_VELOCITY, MAX_ACCELERATION, MIN_ACCELERATION, BUFFER

parser = argparse.ArgumentParser()
parser.add_argument("--map", default="./maps/obstacle.png")
# parser.add_argument("--start", nargs='+', type=float, required=True)
# parser.add_argument("--end", nargs='+', type=float, required=True)

def mpc(obstacles, start, end):

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
    posStack = []
    horizonStack = []
    times = np.array([[0]])

    iteration = 0
    velocitiesX = []
    velocitiesY = []
    accelerationsX = []
    accelerationsY = []

    while (True):
        print("Current loc:", start[:, :])
        t1 = time()
        position = np.copy(start)
        posStack.append(position)

        horizon = []
        for i in x:
            horizon.append(i.value)
        horizonStack.append(horizon)

        x_init.value = start
        # problem.solve(method='dccp', solver=cp.CVXOPT)
        problem.solve(method='dccp')
        t2 = time()
        times = np.vstack((
            times,
            t2 - t1
        ))
        print("--Moving by ", dx[0].value, "/s")
        v = np.copy(dx[0].value)
        velocitiesX.append(v[0][0])
        velocitiesY.append(v[1][0])


        aval = np.copy(a[0].value)
        accelerationsX.append(aval[0][0])
        accelerationsY.append(aval[1][0])

        iteration += 1
        start += B @ dx[0].value
        if np.allclose(start, end) == True:
            break 
  
    simulate(posStack, horizonStack, times, 3, N_HORIZON,
            np.array([0, 0], dtype=np.float64), np.array([800, 800], dtype=np.float64),
            obstacles)

    plt.figure(0)
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_VELOCITY[0], iteration), 'r-.', label = 'Maximum Velocity')
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_VELOCITY[0], iteration), 'r-.', label = 'Minimum Velocity')
    plt.plot(np.arange(1, iteration + 1, 1), velocitiesX, 'g--.', label = 'X Velocity')

    plt.xlabel('Iteration')    
    plt.ylabel('Velocity')    
    plt.title('X Velocity')
    plt.legend(loc="lower left")
    plt.grid()
    plt.show()

    plt.figure(1)
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_VELOCITY[0], iteration), 'r-.', label = 'Maximum Velocity')
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_VELOCITY[0], iteration), 'r-.', label = 'Minimum Velocity')
    plt.plot(np.arange(1, iteration + 1, 1), velocitiesY, 'g--.', label = 'Y Velocity')

    plt.xlabel('Iteration')    
    plt.ylabel('Velocity')    
    plt.title('Y Velocity')
    plt.legend(loc="lower left")
    plt.grid()
    plt.show()

    plt.figure(2)
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_ACCELERATION[0], iteration), 'r-.', label = 'Maximum Acceleration')
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_ACCELERATION[0], iteration), 'r-.', label = 'Minimum Acceleration')
    plt.plot(np.arange(1, iteration + 1, 1), accelerationsX, 'g--.', label = 'X Acceleration')

    plt.xlabel('Iteration')    
    plt.ylabel('Acceleration')    
    plt.title('X Acceleration')
    plt.legend(loc="upper left")
    plt.grid()
    plt.show()

    plt.figure(2)
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_ACCELERATION[0], iteration), 'r-.', label = 'Maximum Acceleration')
    plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_ACCELERATION[0], iteration), 'r-.', label = 'Minimum Acceleration')
    plt.plot(np.arange(1, iteration + 1, 1), accelerationsY, 'g--.', label = 'Y Acceleration')

    plt.xlabel('Iteration')    
    plt.ylabel('Acceleration')    
    plt.title('Y Acceleration')
    plt.legend(loc="upper left")
    plt.grid()
    plt.show()

    return start
   

if __name__ == "__main__":
    args = parser.parse_args()

    start = np.array([0, 0], dtype=np.float64).reshape(-1, 1)
    end = np.array([800, 800], dtype=np.float64).reshape(-1, 1)

    map = load_map(args.map)
    obstacle_list = detect_obstacles(map)
<<<<<<< HEAD
    print(obstacle_list)
    # pos = mpc(obstacle_list, start, end)
=======
    pos = mpc(obstacle_list, start, end)

    
>>>>>>> 7f30859e1e9558f1a2c58f2d8c816cde15433d9e
