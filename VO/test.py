import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

fig=plt.figure()
ax = plt.axes(xlim=(0, 40), ylim=(0, 40))

number_of_particles = 10
data = np.zeros((1000,number_of_particles))
dict_of_circles = {}
for n in range(number_of_particles):
    data[:,n] = [20*(1+np.sin(float(x)/50*n+50)) for x in range(1000)]
    dict_of_circles["circle"+str(n+1)] = plt.Circle((data[0,n],1.0),0.5,fc='b')


def init():
    for n in range(number_of_particles):
        dict_of_circles["circle"+str(n+1)].center = (data[0,n],1)
        ax.add_patch(dict_of_circles["circle"+str(n+1)])
    return dict_of_circles.values()

def animate(i):
    for n in range(number_of_particles):
        dict_of_circles["circle"+str(n+1)].center = (data[i,n],1)
    return dict_of_circles.values()

anim=animation.FuncAnimation(fig,animate,init_func=init,frames=1000,blit=True)

plt.show()