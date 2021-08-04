import cv2

def load_map(map_path):
    map = cv2.imread(map_path, 0)
    return map