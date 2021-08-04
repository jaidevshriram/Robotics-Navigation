import numpy as np

class Line():
    def __init__(self, p0, p1):
        self.p0 = p0
        self.p1 = p1
        
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn