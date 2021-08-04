import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time

from config import N_HORIZON

import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time
import matplotlib.cm as cm

def simulate(cat_states, cat_horiz, t, step_horizon, N, start, end, obstacle_parameters, save=True):
    
    def create_triangle(state=[0,0], h=1, w=0.5, update=False):
            x, y, = state
            triangle = np.array([
                [h,  0  ],
                [0,  w/2],
                [0, -w/2],
                [h,  0  ]
            ]).T
            rotation_matrix = np.array([
                [cos(0), -sin(0)],
                [sin(0),  cos(0)]
            ])

            coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
            if update == True:
                return coords
            else:
                return coords[:3, :]

    def init():
        return path, horizon, current_state, target_state,

    def animate(i):
        # get variables
        try:
            x = cat_states[i][0][0]
            y = cat_states[i][1][0]
        except:
            i = 0
            x = cat_states[i][0][0]
            y = cat_states[i][1][0]

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x)) 
        y_new = np.hstack((path.get_ydata(), y)) 
        path.set_data(x_new, y_new)

        # update horizon
        x_new = []
        for j in cat_horiz[i]:
            if j is None:
                break
            x_new.append(j[0][0])

        y_new = [] 
        for j in cat_horiz[i]:
            if j is None:
                break
            y_new.append(j[1][0])

        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y], update=True))

        return path, horizon, current_state, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(10,10))
    min_scale = min(start[0], start[1]) - 2
    max_scale = max(end[0], end[1]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # print('S: ', start, " E: ", end)

    # create obstacle
    print(obstacle_parameters)
    for obstacle in obstacle_parameters:
        obstacle_circle = plt.Circle((obstacle[0], obstacle[1]), 
                                      obstacle[2], fill=True, color=cm.hot(obstacle[0]/end[0]))
        ax.set_aspect(1)
        ax.add_patch(obstacle_circle) 

    # create lines:
    #   path
    path,  = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon,  = ax.plot([], [], 'x-g', alpha=0.5)
    #   current_state
    current_triangle = create_triangle(start)
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(end)
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=step_horizon*100,
        blit=True,
        repeat=True
    )

    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=5)

    return






