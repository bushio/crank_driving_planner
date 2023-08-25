import numpy as np
from .trajectory_uitl import *

class CurveGenerator:
    def __init__(self, logger):
        self.logger = logger

        self.curve_alpha = 0.5
        self.curve_vel = 1.0
        self.curve_lateral_vel = 5.0
        self.curve_beta_1 = 0.75
        self.curve_beta_2 = 0.75
        self.curve_delta_2 = np.deg2rad(90)

        self.arrival_threthold = 1.5
        self.curve_forward_mergin = 15.0
        self.curve_backward_mergin = 6.0

    def generate_curve_inner(self, new_path, reference_path_array, target_bound, path_index, curve_sign):
        forward_vec = target_bound[path_index + 1] - target_bound[path_index]
        forward_vec_length = np.hypot(forward_vec[0], forward_vec[1])
        forward_vec_norm = forward_vec / forward_vec_length 

        backward_vec = target_bound[path_index - 1] - target_bound[path_index]
        backward_veclength = np.hypot(backward_vec[0], backward_vec[1])
        backward_vec_norm = backward_vec / backward_veclength

        curve_forward_point = target_bound[path_index] + self.curve_forward_mergin * forward_vec_norm
        curve_backward_point = target_bound[path_index] + self.curve_backward_mergin * backward_vec_norm
        
        ## For debug
        self.curve_forward_point = curve_forward_point
        self.curve_backward_point = curve_backward_point
        
        dist_to_right_angle = calcDistancePoitsFromArray(target_bound[path_index], reference_path_array[: , 0:2])
        dist_to_curve_start = calcDistancePoitsFromArray(curve_backward_point, reference_path_array[: , 0:2])
        dist_to_curve_end = calcDistancePoitsFromArray(curve_forward_point, reference_path_array[: , 0:2])

        curve_start_point_idx = dist_to_curve_start.argmin()
        curve_end_point_idx = dist_to_curve_end.argmin()

        self.predicted_goal_pose = reference_path_array[curve_end_point_idx][0:2]
        self.logger.info("[S-crank]: Curve start_ponint_index :{} ,Next point index {}".format(curve_start_point_idx,
                                                                                      curve_end_point_idx
                                                                                      ))
        
        
        middle_point_idx = dist_to_right_angle.argmin()

        quarter_point_idx = int((curve_end_point_idx + middle_point_idx) / 2)
        self.debug_point = reference_path_array[middle_point_idx][0:2]

        ## Calculate new path on curve
        for idx in range(curve_start_point_idx,  middle_point_idx):
            reference_path_array[idx][0:2] += self.curve_beta_1 * forward_vec_norm
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel
            new_path.points[idx].lateral_velocity_mps = self.curve_lateral_vel

            reference_path_array[idx - 1][2] = getInterpolatedYaw(reference_path_array[idx - 1], reference_path_array[idx])
            new_path.points[idx - 1].pose.orientation = getQuaternionFromEuler(yaw=reference_path_array[idx - 1][2])


        rad_d = math.pi / (curve_end_point_idx - middle_point_idx)
        rad = 0.0
        for idx in range(middle_point_idx,  curve_end_point_idx):
            reference_path_array[idx][0:2] += self.curve_beta_2 * backward_vec_norm * np.sin(rad)
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel
            new_path.points[idx].lateral_velocity_mps = self.curve_lateral_vel

            reference_path_array[idx - 1][2] = getInterpolatedYaw(reference_path_array[idx - 1], reference_path_array[idx])
            #reference_path_array[idx - 1][2] += self.curve_delta_2 * curve_sign
            new_path.points[idx - 1].pose.orientation = getQuaternionFromEuler(yaw=reference_path_array[idx - 1][2])

            rad += rad_d
        ## Connect new path and reference path
        """
        connect_vec = reference_path_array[curve_end_point_idx][0:2] - reference_path_array[quarter_point_idx - 1][0:2]
        connect_vec = connect_vec[0:2] / (curve_end_point_idx - quarter_point_idx)
        for idx in range(quarter_point_idx, curve_end_point_idx):
            reference_path_array[idx][0:2] = reference_path_array[idx -1 ][0:2] + connect_vec
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel
            new_path.points[idx].lateral_velocity_mps = self.curve_lateral_vel

            reference_path_array[idx - 1][2] = getInterpolatedYaw(reference_path_array[idx - 1], reference_path_array[idx])
            new_path.points[idx - 1].pose.orientation = getQuaternionFromEuler(yaw=reference_path_array[idx - 1][2])
        """
            

        ## smooting path
        min_path_length = 0.001
        for idx in reversed(range(curve_start_point_idx, curve_end_point_idx)):
            p1 = np.array([new_path.points[idx].pose.position.x, new_path.points[idx].pose.position.y])
            p2 = np.array([new_path.points[idx + 1].pose.position.x, new_path.points[idx + 1].pose.position.y])
            dt = calcDistancePoits(p1, p2)
            if dt < min_path_length:
                del new_path.points[idx]

        goal_distance = calcDistancePoits(self.predicted_goal_pose[0:2], reference_path_array[curve_start_point_idx][0:2])

        self.curve_plot = reference_path_array[curve_start_point_idx:curve_end_point_idx + 1]
        return new_path
    



    def generate_curve_inner2(self, new_path, reference_path_array, outer_bound, outer_bound_index,
                              inner_bound, inner_boundindex, curve_sign):
        #self.logger.debug("inner path index_num {}".format(len(inner_bound)))
        if len(inner_bound) < 4:
            return 

        diag_idx = []
        for idx in range(2, len(inner_bound)):
            cos = getCosFromLines(idx - 1, idx, idx + 1)
            if (cos < 0.7 and cos > 0.4) or (cos < -0.4 and cos > -0.7):
                diag_idx.append(idx)
        self.logger.debug("diagonal count{}".format(len(diag_idx)))

            
        return