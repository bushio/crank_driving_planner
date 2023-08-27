
from autoware_auto_planning_msgs.msg import Trajectory, Path, TrajectoryPoint, PathPoint
from geometry_msgs.msg import Point, Quaternion
import math
import numpy as np

from .numpy_calc_util import *
from .ros_calc_util import *


def getVelocityPointsFromTrajectory(trajctory: Trajectory) -> list:
    points_vel_list = []
    for p in trajctory.points:
        points_vel_list.append(p.longitudinal_velocity_mps)
    return points_vel_list

def getPosesFromTrajectory(trajctory: Trajectory) -> list:
    points_pose_list = []
    for p in trajctory.points: 
            points_pose_list.append(ConvertPoint2List(p.pose.position))
    return points_pose_list

def getAccelPointsFromTrajectory(trajctory: Trajectory) -> list:
    points_accel_list = []
    for p in trajctory.points: 
            points_accel_list.append(p.acceleration_mps2)
    return points_accel_list

def convertPathToTrajectoryPoints(path: Path, 
                                start_idx=0,
                                end_idx=100):
    start_idx = max(start_idx, 0)
    end_idx = min(end_idx, len(path.points) - 1)
    tps = []
    for idx in reversed(range(start_idx, end_idx)):
        p = path.points[idx]
        tp = TrajectoryPoint()
        tp.pose = p.pose
        tp.longitudinal_velocity_mps = p.longitudinal_velocity_mps
        tp.acceleration_mps2 = 0.0
        tps.append(tp)
    return tps


    

