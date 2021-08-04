import numpy as np
import matplotlib.pyplot as plt

from habitat.utils.visualizations import maps

def get_map(sim, size=0.05):

    top_down_map = maps.get_topdown_map(
        sim.pathfinder, 0, meters_per_pixel=size
    )

    recolor_map = np.array(
        [255, 0, 0]
    )

    # plt.imshow(np.uint8(recolor_map[top_down_map]))
    # plt.show()

    return np.uint8(recolor_map[top_down_map])

def make_blobs(map):
    return map