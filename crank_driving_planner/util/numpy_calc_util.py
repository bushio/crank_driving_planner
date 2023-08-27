import numpy as np

def calcDistancePoits(point_a: list, point_b: list) -> float:
    """
    Calculate distance between point_a and point_b.
    """
    if len(point_a) != len(point_b):
        return None
    return np.linalg.norm(np.array(point_a) - np.array(point_b))


def calcDistancePoitsFromArray(point_a: np.array, points: np.array) -> np.array:
    """
    Calculate distance between point_a and mutliple points.
    """
    dist = points[:, 0:2] - point_a[0:2]
    dist = np.hypot(dist[:, 0], dist[:, 1])
    return dist


def getNearestPointIndex(point: np.array, points: np.array) -> int:
    """
    Find th nearest point from points array.
    """
    dist =  calcDistancePoitsFromArray(point, points)
    return dist.argmin()


def getInterpolatedYaw(p1, p2) -> np.array:
    """
    Calc yaw from two points.
    """
    diff_x = p2[0] - p1[0]
    diff_y = p2[1] - p1[1]
    return np.arctan2(diff_y, diff_x)


def getCrossPoint(p1: np.array, vec1: np.array, p2: np.array, vec2: np.array) -> np.array:
    """
    Get cross point.
    """
    d = vec1[0] * vec2[1] - vec2[0] * vec1[1]
    if d == 0:
         return None
    sn = vec2[1] * (p2[0] - p1[0]) - vec2[0] * (p2[1] - p1[1])
    print(sn / d)
    return np.array([p1[0] + vec1[0] * (sn / d), p1[1] + vec1[1] * (sn / d)])


def getNormVec(p: np.array, q: np.array) -> np.array:
    """
    Get normalize vector from two points
    """
    vec = q - p
    vec /= np.hypot(vec[0], vec[1])
    return vec


def getTriangleSize(p1: np.array, p2: np.array, p3: np.array) -> float:
    """
    Get triangle size from three points
    """
    return 0.5 * abs(p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]))

def getCosFromLines(p1: np.array, p2: np.array, p3: np.array) -> float:
    """
    Get cos from three points
    """
    vec_1 = p1 - p2
    vec_2= p3 - p2
    d1  = np.hypot(vec_1[0], vec_1[1])
    d2  = np.hypot(vec_2[0], vec_2[1])
    cos = np.dot(vec_1, vec_2) / (d1 * d2)
    return cos

def getRoadWidth(inner_bound: np.array, outer_bound: np.array, diag_idx: int, sharp_idx: int) -> float:
    """
    Calculate the width of road afeter curve
    """

    inner_backward_vec = getNormVec(inner_bound[diag_idx], inner_bound[diag_idx - 1])
    outer_forward_vec = getNormVec(outer_bound[sharp_idx], outer_bound[sharp_idx + 1])
    inner_finish_point = inner_bound[diag_idx + 1]
    outer_finish_point = getCrossPoint(inner_bound[diag_idx + 1], 
                                            -1 *inner_backward_vec,
                                            outer_bound[sharp_idx],
                                            outer_forward_vec)
    return calcDistancePoits(outer_finish_point, inner_finish_point)