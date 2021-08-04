import numpy as np

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex

def to_list_of_tuples(iterable):
    return [(float(p.x()), float(p.y())) for p in iterable]