from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
from simulate import simulate

from helpers import make_obstacles

# setting matrix_weights' variables
Q_x = 100
Q_y = 100
Q_theta = 2000
R1 = 1
R2 = 1
R3 = 1
R4 = 1

step_horizon = 1   
N = 1              
rob_radius = 0.25    
wheel_radius = 0.07  
Lx = 0.3             
Ly = 0.3             
sim_time = 200       

# specs
x_init = 0
y_init = 0

x_target = 20
y_target = 20

v_max = 500 * (2*pi/60)     #rad/sec
v_min = -500 * (2*pi/60)    #rad/sec

# adding constraint for collision avoidance
obstacle_x = 10  # x_dir
obstacle_y = 10  # y_dir
obstacle_r = 2.5 # radius   

def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


def DM2Arr(dm):
    return np.array(dm.full())

def make_solver():

    # state symbolic variables
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(
        x,
        y,
        theta
    )
    n_states = states.numel()

    # control symbolic variables
    V_a = ca.SX.sym('V_a')
    V_b = ca.SX.sym('V_b')
    V_c = ca.SX.sym('V_c')
    V_d = ca.SX.sym('V_d')
    controls = ca.vertcat(
        V_a,
        V_b,
        V_c,
        V_d
    )
    n_controls = controls.numel()

    # matrix containing all states over all time steps +1 (each column is a state vector)
    X = ca.SX.sym('X', n_states, N+1)

    # matrix containing all control actions over all time steps (each column is an action vector)
    U = ca.SX.sym('U', n_controls, N)

    # coloumn vector for storing initial state and target state
    P = ca.SX.sym('P', n_states + n_states)

    # state weights matrix (Q_X, Q_Y, Q_THETA)
    Q = ca.diagcat(Q_x, Q_y, Q_theta)

    # controls weights matrix
    R = ca.diagcat(R1, R2, R3, R4)

    # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
    rot_3d_z = ca.vertcat(
        ca.horzcat(cos(theta), -sin(theta), 0),
        ca.horzcat(sin(theta),  cos(theta), 0),
        ca.horzcat(         0,           0, 1)
    )

    J = (wheel_radius/4) * ca.DM([
        [         1,         1,          1,         1],
        [        -1,         1,          1,        -1],
        [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]
    ])

    RHS = rot_3d_z @ J @ controls
    # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
    f = ca.Function('f', [states, controls], [RHS])

    cost_fn = 0  # cost function
    g = X[:, 0] - P[:n_states]  # constraints in the equation

    # runge kutta 
    for k in range(N):
        st = X[:, k]
        con = U[:, k]
        cost_fn = cost_fn \
            + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) \
            + con.T @ R @ con
        st_next = X[:, k+1]
        k1 = f(st, con)
        k2 = f(st + step_horizon/2*k1, con)
        k3 = f(st + step_horizon/2*k2, con)
        k4 = f(st + step_horizon * k3, con)
        st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        g = ca.vertcat(g, st_next - st_next_RK4)

    print('g1'+g.dim()+'\n')

    # appending inequality constraints to g
    for k in range(N+1):
        g = ca.vertcat(g, -ca.sqrt( (X[0,k]-obstacle_x)**2 + (X[1,k]-obstacle_y)**2 )
                    + (rob_radius + obstacle_r))

    print('g2'+g.dim()+'\n')

    OPT_variables = ca.vertcat(
        X.reshape((-1, 1)),   
        U.reshape((-1, 1))
    )
    nlp_prob = {
        'f': cost_fn,
        'x': OPT_variables,
        'g': g,
        'p': P
    }

    opts = {
        'ipopt': {
            'max_iter': 200,
            'print_level': 0,
            'acceptable_tol': 1e-8,
            'acceptable_obj_change_tol': 1e-6
        },
        'print_time': 0
    }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
    return solver

lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N , 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N , 1))

lbx[0: n_states*(N+1): n_states] = -ca.inf     # X lower bound
lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = ca.inf      # X upper bound
ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

lbx[n_states*(N+1):] = v_min                   # v lower bound for all V
ubx[n_states*(N+1):] = v_max                   # v upper bound for all V

# boundries
lbg = ca.DM.zeros((n_states*(N+1)+(N+1), 1))
ubg = ca.DM.zeros((n_states*(N+1)+(N+1), 1))

# equality constraints
lbg[0: n_states*(N+1)] = 0
ubg[0: n_states*(N+1)] = 0

# inequality constraints
lbg[n_states*(N+1): n_states*(N+1)+(N+1)] = -ca.inf
ubg[n_states*(N+1): n_states*(N+1)+(N+1)] = 0

print('lbx'+lbx.dim()+'\n')
print('ubx'+ubx.dim()+'\n')
print('lbg'+lbg.dim()+'\n')
print('ubg'+ubg.dim()+'\n')

args = {
    'lbg': lbg,     
    'ubg': ubg,     
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0
state_init = ca.DM([x_init, y_init, theta_init])          # initial state
state_target = ca.DM([x_target, y_target, theta_target])  # target state

t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))          # initial control
X0 = ca.repmat(state_init, 1, N+1)         # initial state full


vo_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])



if __name__ == '__main__':
    main_loop = time()  # return time in sec
    obstacle_list = make_obstacles()
    while (ca.norm_2(state_init - state_target) > 1e-1) and (vo_iter * step_horizon < sim_time):
        t1 = time()
        args['p'] = ca.vertcat(
            state_init,    # current state
            state_target   # target state
        )
        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )

        solver = make_solver()

        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))

        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)

        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        print(vo_iter)
        print(t2-t1)
        times = np.vstack((
            times,
            t2-t1
        ))

        vo_iter = vo_iter + 1

    main_loop_time = time()
    ss_error = ca.norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    simulate(cat_states, cat_controls, times, step_horizon, N,
             np.array([x_init, y_init, theta_init, x_target, y_target, theta_target]),
             np.array([obstacle_x, obstacle_y, obstacle_r]), save=True)