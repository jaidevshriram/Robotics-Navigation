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

def form_problem(time, obstacles, start, end):
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
    x_init.value = start
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

            if time + i >= len(j['xs']):
                pos = [j['xs'][len(j['xs'])-1], j['ys'][len(j['xs'])-1]]
            else:
                pos = [j['xs'][len(j['xs'])-1], j['ys'][len(j['xs'])-1]]

            r = j['r']
            obs_pos = np.array(pos).reshape(-1, 1)

            constraints += [cp.norm(x[i] - obs_pos) >= r**2]

            # if time + i >= len(j['xs']):
            #     continue
            # # print("CONSTRAINT", j['xs'][time + i], j['ys'][time + i])

            # r = j['r']
            # pos = [j['xs'][time + i], j['ys'][time + i]]

            # try:
            #     v_obs = [j['xs'][time + i + 1] - j['xs'][time + i], j['ys'][time + i + 1] - j['ys'][time + i]]
            # except:
            #     v_obs = [0, 0]

            # v_obs = np.array(v_obs).reshape(-1, 1)
            # obs_pos = np.array(pos).reshape(-1, 1)

            # m = dx[i].T * (x[i] - obs_pos)
            # n = v_obs.T * (obs_pos-x[i])

            # u = -cp.norm(x[i]-obs_pos) + r**2
            # p = cp.norm(dx[i])
            # q = -2*(dx[i].T @ v_obs)
            # r = cp.norm(v_obs)

            # A = m**2 + (u*p)
            # B = 2*m*n + (u*q)
            # C = n**2 + (u*r)

            # constraints += [A + B + C <= 0] 
       
    problem = cp.Problem(cp.Minimize(objective), constraints)

    return problem, x, dx, a

def checkCollision(start, x, dx, a, obstacles, time):
    B = np.eye(2)

    for i in range(len(obstacles)):
        obs = obstacles[i]
        

def mpc(obstacles_all, start, end):

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
        print("Time", iteration, "Current loc:", start.flatten())
        print("Obstacle Pos", obstacles_all[0]['xs'][min(iteration, len(obstacles_all[0]['xs'])-1)], obstacles_all[0]['ys'][min(iteration, len(obstacles_all[0]['xs'])-1)])
        t1 = time()
        position = np.copy(start)
        posStack.append(position)

        problem, x, dx, a = form_problem(iteration, obstacles_all, start, end)

        horizon = []
        for i in x:
            horizon.append(i.value)
        horizonStack.append(horizon)

        problem.solve(method='dccp', solver=cp.CVXOPT)
        # problem.solve(method='dccp')
        t2 = time()
        times = np.vstack((
            times,
            t2 - t1
        ))
        print("--Moving by ", np.round(dx[0].value.flatten(), 2), "/s")
        v = np.copy(dx[0].value)
        velocitiesX.append(v[0][0])
        velocitiesY.append(v[1][0]) 

        A = B = np.eye(2)

        aval = np.copy(a[0].value)
        accelerationsX.append(aval[0][0])
        accelerationsY.append(aval[1][0])

        iteration += 1

        isCol = checkCollision(start, x, dx, a)

        start += B @ dx[0].value
        if np.allclose(start, end) == True:
            break
  
    # plt.plot(horizon[:, 0], horizon[:, 1])
    # plt.show()
    # simulate(posStack, horizonStack, times, 3, N_HORIZON,
            # np.array(start, dtype=np.float64), np.array(end, dtype=np.float64),
            # [])

    # plt.figure(0)
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_VELOCITY[0], iteration), 'r-.', label = 'Maximum Velocity')
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_VELOCITY[0], iteration), 'r-.', label = 'Minimum Velocity')
    # plt.plot(np.arange(1, iteration + 1, 1), velocitiesX, 'g--.', label = 'X Velocity')

    # plt.xlabel('Iteration')    
    # plt.ylabel('Velocity')    
    # plt.title('X Velocity')
    # plt.legend(loc="lower left")
    # plt.grid()
    # plt.show()

    # plt.figure(1)
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_VELOCITY[0], iteration), 'r-.', label = 'Maximum Velocity')
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_VELOCITY[0], iteration), 'r-.', label = 'Minimum Velocity')
    # plt.plot(np.arange(1, iteration + 1, 1), velocitiesY, 'g--.', label = 'Y Velocity')

    # plt.xlabel('Iteration')    
    # plt.ylabel('Velocity')    
    # plt.title('Y Velocity')
    # plt.legend(loc="lower left")
    # plt.grid()
    # plt.show()

    # plt.figure(2)
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_ACCELERATION[0], iteration), 'r-.', label = 'Maximum Acceleration')
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_ACCELERATION[0], iteration), 'r-.', label = 'Minimum Acceleration')
    # plt.plot(np.arange(1, iteration + 1, 1), accelerationsX, 'g--.', label = 'X Acceleration')

    # plt.xlabel('Iteration')    
    # plt.ylabel('Acceleration')    
    # plt.title('X Acceleration')
    # plt.legend(loc="upper left")
    # plt.grid()
    # plt.show()

    # plt.figure(2)
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MAX_ACCELERATION[0], iteration), 'r-.', label = 'Maximum Acceleration')
    # plt.plot(np.arange(1, iteration + 1, 1), np.repeat(MIN_ACCELERATION[0], iteration), 'r-.', label = 'Minimum Acceleration')
    # plt.plot(np.arange(1, iteration + 1, 1), accelerationsY, 'g--.', label = 'Y Acceleration')

    # plt.xlabel('Iteration')    
    # plt.ylabel('Acceleration')    
    # plt.title('Y Acceleration')
    # plt.legend(loc="upper left")
    # plt.grid()
    # plt.show()

    return start
   
def make_obstacles():
    xs = np.linspace(0, 10, 10)
    ys = np.linspace(0, 10, 10)
    r = 1

    obstacles = [{
        'r': r,
        'xs': xs,
        'ys': ys,
    }]

    return obstacles

if __name__ == "__main__":
    args = parser.parse_args()

    start = np.array([0, 5], dtype=np.float64).reshape(-1, 1)
    end = np.array([10, 5], dtype=np.float64).reshape(-1, 1)

    # map = load_map(args.map)
    # obstacle_list = detect_obstacles(map)

    obstacle_list = make_obstacles()
    print(obstacle_list)
    # pos = mpc(obstacle_list, start, end)
    pos = mpc(obstacle_list, start, end)