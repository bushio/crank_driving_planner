import numpy as np
import copy
from .trajectory_uitl import *
from autoware_auto_planning_msgs.msg import Path, PathPoint

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

        self.enable_planning_flag = True
        self.inner_finish_mergin = 5.0
        self.inner_start_mergin = 2.0
        self.margin_idx = 10


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
    



    def generate_curve_inner2(self, reference_path, reference_path_array : np.array, outer_bound : np.array, outer_bound_index: int,
                              inner_bound : np.array, inner_bound_index: int, curve_sign: int):
        #self.logger.info("inner bound num{}".format(len(inner_bound)))
        if len(inner_bound) < 4:
            return 

        for idx in range(2, len(inner_bound)):
            inner_cos= getCosFromLines(inner_bound[idx - 2], inner_bound[idx - 1], inner_bound[idx])

            #self.logger.info("inner cos {}".format(inner_cos))
            if (inner_cos < 0.85 and inner_cos > 0.4) or (inner_cos < -0.4 and inner_cos > -0.85):
                diag_idx = idx -1 
                break

        for idx in range(2, len(outer_bound)):
            outer_cos= getCosFromLines(outer_bound[idx - 2], outer_bound[idx - 1], outer_bound[idx])
            #self.logger.info("outer cos {}".format(outer_cos))
            if (outer_cos < 0.2 and outer_cos > -0.2):
                sharp_index = idx -1
                break
        
        if  self.enable_planning_flag:
            self.enable_planning_flag = False

            self.outer_bound = outer_bound
            self.inner_bound = inner_bound
            self.diag_idx = diag_idx
            self.sharp_index = sharp_index
        
            ## Calculate vec 
            outer_backward_vec = getNormVec(self.outer_bound[self.sharp_index], self.outer_bound[self.sharp_index - 1])
            inner_backward_vec = getNormVec(self.inner_bound[self.diag_idx], self.inner_bound[self.diag_idx - 1])
            outer_forward_vec = getNormVec(self.outer_bound[self.sharp_index], self.outer_bound[self.sharp_index + 1])
            inner_forward_vec = getNormVec(self.inner_bound[self.diag_idx + 1], self.inner_bound[self.diag_idx + 2])

            ##  Get sharp_point
            inner_sharp_point = getCrossPoint(self.inner_bound[self.diag_idx + 1], 
                                            -1 *inner_backward_vec,
                                            self.inner_bound[self.diag_idx + 2],
                                            -1* inner_forward_vec)
            outer_sharp_point = self.outer_bound[self.sharp_index]
            
            inner_start_point = self.inner_bound[self.diag_idx]
            inner_finish_point = self.inner_bound[self.diag_idx + 1]
            outer_finish_point = getCrossPoint(self.inner_bound[self.diag_idx + 1], 
                                            -1 *inner_backward_vec,
                                            self.outer_bound[self.sharp_index],
                                            outer_forward_vec)
            
            inner_finish_mergin_point = inner_finish_point + self.inner_finish_mergin * inner_forward_vec
            inner_start_mergin_point = inner_start_point + self.inner_start_mergin * inner_backward_vec

            
            road_width = calcDistancePoits(outer_finish_point, inner_finish_point)
            
            curve_start_idx = getNearestPointIndex(inner_start_point, reference_path_array[:, 0:2])
            curve_start_margin_idx = getNearestPointIndex(inner_start_mergin_point , reference_path_array[:, 0:2])

            curve_end_idx = getNearestPointIndex(inner_finish_point, reference_path_array[:, 0:2])
            curve_end_margin_idx = getNearestPointIndex(inner_finish_mergin_point, reference_path_array[:, 0:2])

            self.logger.info("Road width {}".format(road_width))

            if curve_sign < 0:
                ## Turn right
                current_rad = math.pi
            else:
                ## Turn left
                current_rad = math.pi

            if road_width > 3.0:
                R = 4.0
                self.alpha = 0.7
            elif road_width > 2.5:
                 R = 4.5
                 self.alpha = 0.6
            else:
                R = 3.0

            ## Set curve start and end
            self.curve_start_idx = curve_start_margin_idx
            self.curve_end_idx = curve_end_margin_idx

            curve_start_point = copy.deepcopy(reference_path_array[self.curve_start_idx, 0:2])
            finish_point = 0.5 * (outer_finish_point - inner_finish_point) + inner_finish_point
            curve_end_point = copy.deepcopy(reference_path_array[self.curve_end_idx, 0:2])
            


            ## Calculate small path
            print("rad",road_width, self.alpha )
            d_rad = (math.pi * self.alpha) /(self.curve_end_idx - self.curve_start_idx)
            small_path = copy.deepcopy(reference_path_array)
            raidus_vec = inner_forward_vec * R
            center_point = curve_start_point[0:2] + raidus_vec
            
            for idx in range(self.curve_start_idx, self.curve_end_idx):
                rotated_vec = np.array([raidus_vec[0] * np.cos(current_rad) - raidus_vec[1] * np.sin(current_rad),
                                    raidus_vec[0] * np.sin(current_rad) + raidus_vec[1] * np.cos(current_rad)])
                rotated_vec /= np.hypot(rotated_vec[0],rotated_vec[1])
                small_path[idx][0] = center_point[0] + rotated_vec[0] * R
                small_path[idx][1] = center_point[1] + rotated_vec[1] * R
                current_rad +=  d_rad * curve_sign

            #
            small_path, add_idx = self._connect_path( 
                    small_path , 
                    self.curve_end_idx - 1 ,
                    self.curve_end_idx + self.margin_idx,
                    min_path_point_dist = 0.5)
            self.logger.info("Path points num {}".format(len(small_path)))
            
            """
            small_path = self._smooth_path(
                small_path,
                self.curve_start_idx,
                self.curve_end_idx + self.margin_idx
            )
            """
            self.predicted_goal_pose = finish_point
            ## Cut path
            small_path_cut = small_path[self.curve_start_idx: self.curve_end_idx + add_idx]

            self.logger.info("Path points num {}".format(len(small_path)))
            new_path = self._store_array_into_path(
                           reference_path,
                           small_path_cut,
                           self.curve_start_idx)
            
            self.curve_plot = small_path_cut
            self.debug_point = np.array([
                                         center_point,
                                         curve_start_point,
                                         #inner_finish_point,
                                         #outer_finish_point,
                                         #inner_start_mergin_point,
                                         #inner_finish_mergin_point
                                         ])
            return new_path
        
    def reset_enable_planning(self):
        self.enable_planning_flag = True
        
    def _store_array_into_path(self,
                           new_path,
                           predicted_path_array,
                           start_idx
                           ):
        start_idx = max(start_idx, 1)
        reference_points = copy.deepcopy(new_path.points)
        z_value = reference_points[0].pose.position.z
        new_path.points = []
        ## store refrence path into new_path
        for idx in range(start_idx):
                new_path.points.append(reference_points[idx])
        
        ## store predicted path into new_path
        for idx in range(len(predicted_path_array)):
                pose = predicted_path_array[idx]
                point = getPathPoint(pose,
                                    self.curve_vel, 
                                    self.curve_lateral_vel,
                                    z_value=z_value)
                yaw = getInterpolatedYawFromPoint(new_path.points[-1], point)
                point.pose.orientation = getQuaternionFromEuler(yaw=yaw)
                new_path.points.append(point)
        return new_path

    def _connect_path(self, 
                      reference_path_array, 
                      connect_start_idx,
                      connect_end_idx,
                      min_path_point_dist = 0.5):
        connect_end_idx = min(connect_end_idx, len(reference_path_array) - 1)
        # Connect_path
        connect_vec = getNormVec(reference_path_array[connect_start_idx, 0:2], reference_path_array[connect_end_idx, 0:2]) * min_path_point_dist
        connect_dist = calcDistancePoits(reference_path_array[connect_start_idx, 0:2], reference_path_array[connect_end_idx, 0:2])
        connect_point_num = int(connect_dist / 0.5)
        for idx in range(connect_point_num):
            offset_idx = connect_start_idx + idx
            if offset_idx > len(reference_path_array) - 1:
                break
            reference_path_array[offset_idx][0:2] = reference_path_array[offset_idx -1][0:2] + connect_vec
        return reference_path_array,idx

    def _smooth_path(self, 
                    reference_path_array, 
                    start_idx,
                    end_idx,
                    min_path_length = 0.001):
        # Connect_path
        end_idx = min(len(reference_path_array) - 2, end_idx)
        for idx in reversed(range(start_idx, end_idx)):
            dt = calcDistancePoits(reference_path_array[idx, 0:2], reference_path_array[idx + 1, 0:2])
            if dt < min_path_length:
                del reference_path_array[idx]
        return reference_path_array