import numpy as np
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
from util import *


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

def get_sharp_point(bound):
    sharp_index = None
    if len(bound) < 3:
        return sharp_index

    for idx in range(2, len(bound)):
        cos= getCosFromLines(bound[idx - 2], bound[idx - 1], bound[idx])
        if (cos < 0.2 and cos > -0.2):
            sharp_index = idx -1
            break
    return sharp_index

def get_diag_point(bound):
    diag_idx = None
    if len(bound) < 3:
        return diag_idx
    for idx in range(2, len(bound)):
        cos= getCosFromLines(bound[idx - 2], bound[idx - 1], bound[idx])
        if (cos < 0.85 and cos > 0.4) or (cos < -0.4 and cos > -0.85):
            diag_idx = idx -1 
            break
    return diag_idx
