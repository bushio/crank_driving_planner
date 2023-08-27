import numpy as np
from autoware_auto_planning_msgs.msg import Path, PathPoint

from .numpy_calc_util import *
from .ros_calc_util import *

def getPathPoint(pose_array: np.array, longitudinal_vel: float, lateral_vel: float, z_value=0.0):
    path_point = PathPoint()
    path_point.pose.position.x = pose_array[0]
    path_point.pose.position.y = pose_array[1]
    path_point.pose.position.z = z_value
    path_point.longitudinal_velocity_mps  = longitudinal_vel
    path_point.lateral_velocity_mps  = lateral_vel
    return path_point

##
def ConvertPath2Array(path: Path) -> np.array:
    new_path = np.empty((0,3))
    for idx in range(len(path.points)):
        x = path.points[idx].pose.position.x
        y = path.points[idx].pose.position.y
        yaw = getYawFromQuaternion(path.points[idx].pose.orientation)
        new_path = np.vstack([new_path, np.array([x, y, yaw])])
    return new_path