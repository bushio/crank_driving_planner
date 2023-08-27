
import numpy as np
from .util import *
from .config import DWA_Config
from .predictor import DynamicWindowApproach

class PathPredictor:
    def __init__(self, logger, clock):
        self.logger = logger
        self.get_clock = clock

        self.dwa_config = DWA_Config()
        self.predictor = DynamicWindowApproach(self.dwa_config)
        
        self.min_point_dist = 2.0
        self.vel = 0.5
        self.min_diff_for_update = 0.1 **3

        self.ego_pose_predicted = None
        self.predicted_traj = None
        self.output_traj = None

    def predict_path_by_dwa(self, reference_path, ego_pose_array, predicted_goal_pose, object_pose, left_bound, right_bound,
                            current_left_path_index, current_right_path_index):
        new_path = reference_path

        if self.ego_pose_predicted is None:
            self.logger.info("Initialize the ego pose {}".format(np.rad2deg(ego_pose_array)))
            self.ego_pose_predicted = np.array([ego_pose_array[0],
                                                ego_pose_array[1], 
                                                ego_pose_array[2],
                                                0.0,
                                                0.0,
                                                ])
        elif (abs(self.ego_pose_predicted[0] -ego_pose_array[0])< self.min_diff_for_update) or \
            (abs(self.ego_pose_predicted[1] - ego_pose_array[1])  < self.min_diff_for_update):
            self.logger.info("The ego pose has not updated yet...")
            return self.output_traj

        else:
            self.logger.info("Update the ego pose {}".format(ego_pose_array))
            self.logger.info("goal pose {}".format(predicted_goal_pose))
            self.ego_pose_predicted[0] = ego_pose_array[0]
            self.ego_pose_predicted[1] = ego_pose_array[1]
            self.ego_pose_predicted[2] = ego_pose_array[2]


        self.logger.info("Predict ego_pose {}".format(self.ego_pose_predicted))
        u, predicted_traj = self.predictor.get_next_step(
                self.ego_pose_predicted, 
                predicted_goal_pose, 
                object_pose, 
                left_bound[current_left_path_index: current_left_path_index + 2], 
                right_bound[current_right_path_index: current_right_path_index + 2])

        if len(predicted_traj) == 0:
            return

        self.ego_pose_predicted[3] = u[0]
        self.ego_pose_predicted[4] = u[1]
        output_traj = Trajectory()
        output_traj.points = convertPathToTrajectoryPoints(new_path, len(predicted_traj) + 1)
        output_traj.header = new_path.header
        output_traj.header.stamp = self.get_clock.now().to_msg()

        point_dist = 0.0
        calc_point = self.ego_pose_predicted[0:2]
        
        ignore_point = 0
        output_traj.points[0].pose.position.x = ego_pose_array[0]
        output_traj.points[0].pose.position.y = ego_pose_array[1]
        output_traj.points[0].longitudinal_velocity_mps = self.vel
        for idx in range(len(predicted_traj) -1):
            dt = calcDistancePoits(predicted_traj[idx][0:2], calc_point[0:2])
            point_dist += dt
            if point_dist > self.min_point_dist:
                output_traj.points[idx + 1].pose.position.x = predicted_traj[idx][0]
                output_traj.points[idx + 1].pose.position.y = predicted_traj[idx][1]
                output_traj.points[idx].pose.orientation = getQuaternionFromEuler(yaw=predicted_traj[idx][2])
                output_traj.points[idx + 1].longitudinal_velocity_mps = self.vel
                calc_point = predicted_traj[idx][0:2]
                point_dist = 0.0
            else:
                ignore_point += 1

        for _ in range(ignore_point):
            output_traj.points.pop(-1)
        
        self.predicted_traj = predicted_traj
        self.output_traj = output_traj
        return output_traj


