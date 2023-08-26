
from autoware_auto_planning_msgs.msg import Trajectory, Path, TrajectoryPoint
from geometry_msgs.msg import Point, Quaternion
import math
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


def calcDistancePoitsFromArray(point_a: np.array, points: np.array) -> np.array:
    dist = points[:, 0:2] - point_a[0:2]
    dist = np.hypot(dist[:, 0], dist[:, 1])
    return dist


def getNearestPointIndex(point: np.array, points: np.array) -> int:
    dist =  calcDistancePoitsFromArray(point, points)
    return dist.argmin()


def ConvertPoint2List(p) -> np.array:
    yaw = getYawFromQuaternion(p.orientation)
    return np.array([p.position.x, p.position.y, yaw])


def calcDistancePoits(point_a: list, point_b: list) -> float:
    """
    Calculate distance between point_a and point_b.
    """
    if len(point_a) != len(point_b):
        return None
    return np.linalg.norm(np.array(point_a) - np.array(point_b))


def ConvertPointSeq2Array(points: list) -> np.array:
    k = []
    for i in range(len(points)):
        k.append([points[i].x, points[i].y])
    return np.array(k)


def ConvertPath2Array(path: Path) -> np.array:
    new_path = np.empty((0,3))
    for idx in range(len(path.points)):
        x = path.points[idx].pose.position.x
        y = path.points[idx].pose.position.y
        yaw = getYawFromQuaternion(path.points[idx].pose.orientation)
        new_path = np.vstack([new_path, np.array([x, y, yaw])])
    return new_path


def getYawFromQuaternion(orientation):
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def convertPathToTrajectoryPoints(path: Path, point_num: int):
    tps = []
    for idx in reversed(range(point_num)):
        p = path.points[idx]
        tp = TrajectoryPoint()
        tp.pose = p.pose
        tp.longitudinal_velocity_mps = p.longitudinal_velocity_mps
        tp.acceleration_mps2 = 0.0
        tps.append(tp)
    return tps

def getQuaternionFromEuler(roll: float =0, pitch :float =0 ,yaw :float =0) -> Quaternion:
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

def getInterpolatedYaw(p1, p2):
    diff_x = p2[0] - p1[0]
    diff_y = p2[1] - p1[1]
    return np.arctan2(diff_y, diff_x)


def getCosFromLines(p1, p2, p3):
        vec_1 = p1 - p2
        vec_2= p3 - p2
        d1  = np.hypot(vec_1[0], vec_1[1])
        d2  = np.hypot(vec_2[0], vec_2[1])
        cos = np.dot(vec_1, vec_2) / (d1 * d2)
        return cos

def getCrossPoint(p1, vec1, p2, vec2):
    d = vec1[0] * vec2[1] - vec2[0] * vec1[1]
    if d == 0:
         return None
    sn = vec2[1] * (p2[0] - p1[0]) - vec2[0] * (p2[1] - p1[1])
    print(sn / d)
    return np.array([p1[0] + vec1[0] * (sn / d), p1[1] + vec1[1] * (sn / d)])
    
def getNormVec(p, q):
     vec = q - p
     vec /= np.hypot(vec[0], vec[1])
     return vec

def getTriangleSize(p1, p2, p3):
     return 0.5 * abs(p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]))