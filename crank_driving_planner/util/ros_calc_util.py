import numpy as np
import math 
from geometry_msgs.msg import Point, Quaternion

def ConvertPoint2List(p) -> np.array:
    """
    Convert Point.msg into numpu.array
    """
    yaw = getYawFromQuaternion(p.orientation)
    return np.array([p.position.x, p.position.y, yaw])


def getYawFromQuaternion(orientation: Quaternion):
    """
    Calculate yaw from quaternion
    """
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return np.arctan2(siny_cosp, cosy_cosp)

def ConvertPointSeq2Array(points: list) -> np.array:
    """
    Convert the list of Point.msg  into numpy array
    """
    k = []
    for i in range(len(points)):
        k.append([points[i].x, points[i].y])
    return np.array(k)

def getQuaternionFromEuler(roll: float =0, pitch :float =0 ,yaw :float =0) -> Quaternion:
    """
    Convert Euler into Quaternion
    """
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

def getInterpolatedYawFromPoint(p1, p2):
    diff_x = p2.pose.position.x - p1.pose.position.x
    diff_y = p2.pose.position.y - p1.pose.position.y
    return np.arctan2(diff_y, diff_x)

