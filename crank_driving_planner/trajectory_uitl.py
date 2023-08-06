
from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point
import numpy as np


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

# ToDO: Change to BinarySearch
def getNearestPointIndex(point, points) -> int:
    d = calcDistancePoits(point, points[0])
    idx = 0
    for i, p in enumerate(points[1:]):
        d_ = calcDistancePoits(point, p)
        if d_ < d:
            d = d_
            idx = i
    return idx 

def ConvertPoint2List(p :Point) ->list:
    return [p.x, p.y, p.z]


def calcDistancePoits(point_a: list, point_b: list) -> float:
    """
    Calculate distance between point_a and point_b.
    """
    if len(point_a) != len(point_b):
        print("Shape of the poits must be same.")
        return 0.0
    return np.linalg.norm(np.array(point_a) - np.array(point_b))