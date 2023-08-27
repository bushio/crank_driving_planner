import numpy as np
import sys
from ..util import *

def get_nearest_path_idx(ego_pose, bound, next_path_threshold=5.0):
    diff = calcDistancePoitsFromArray(ego_pose, bound)
    current_path_index = diff.argmin()
    
    next_path_index = None
    for idx in range(len(diff)):
        if diff[idx] > next_path_threshold:
            next_path_index = idx
            break

    if next_path_index is None or next_path_index == 0:
        next_path_index = current_path_index + 1
    
    return current_path_index, next_path_index