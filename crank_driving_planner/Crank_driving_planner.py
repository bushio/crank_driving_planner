import time
import rclpy
import numpy as np
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory, Path
from geometry_msgs.msg import AccelWithCovarianceStamped, Point
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry

from .trajectory_uitl import *
from .predicted_objects_info import PredictedObjectsInfo

## For plot
from .debug_plot import PlotMarker
# Dynamic Window Approach
from .config import Config
from .dynamic_window_approach import DynamicWindowApproach

NANO_SECONDS = (1 / 1000)**3

class CrankDrigingPlanner(Node):
    def __init__(self):
        super().__init__('CrankDrigingPlanner')
        self.get_logger().info("Start CrankDrigingPlanner")
        
        ## Reference trajectory subscriber. Remap "/planning/scenario_planning/lane_driving/behavior_planning/path" ##
        self.create_subscription(Path ,"~/input/path", self.onTrigger, 10)

        ## Accrel subscriber ##
        self.create_subscription(AccelWithCovarianceStamped, "~/input/acceleration", self.onAcceleration, 10)

        ## Vehicle odometry subscriber ##
        self.create_subscription(Odometry, "~/input/odometry", self.onOdometry, 10)

        ## Vehicle odometry subscriber ##
        self.create_subscription(Odometry, "~/input/odometry", self.onOdometry, 10)

        # Predicted objects subscriber
        self.create_subscription(PredictedObjects, "~/input/perception", self.onPerception, 10)

        # Path publisher. Remap "/planning/scenario_planning/lane_driving/path" ##
        self.pub_path_ = self.create_publisher(Path, 
                                               "~/output/path", 
                                                10)
        # trajectory publisher. Remap "/planning/scenario_planning/lane_driving/trajectory" ##
        #self.pub_traj_ = self.create_publisher(Trajectory, "/planning/scenario_planning/lane_driving/trajectory", 10)
        self.pub_traj_ = self.create_publisher(Trajectory, "~/output/trajectory", 10)

        # Initialize input ##
        self.reference_path = None
        self.current_accel = None
        self.current_odometry = None
        self.crrent_longitudinal_velocity = 0.0
        self.ego_pose = None
        self.dynamic_objects = None
        self.left_bound  = None
        self.right_bound  = None
        self.current_left_path_index = None
        self.current_right_path_index = None

        self.vehicle_state = "initial"
        self.before_exec_time = -9999 * NANO_SECONDS
        self.duration = 10.0

        self.planning_traj_pub = None
        self.planning_path_pub = None


        ## Check stop time
        self.stop_time = 0.0
        self.stop_duration = 7

        
        

        self.animation_flag = True
        self.debug = False
        
        if self.animation_flag:
            self.plot_marker = PlotMarker()
            self.curve_plot = None
            self.curve_backward_point = None
            self.curve_forward_point = None
        ## Dynamic Window Approach
        self.use_dwa = False
        if self.use_dwa:
            self.dwa_config = Config()
            self.predictor = DynamicWindowApproach(self.dwa_config)
        self.ego_pose_predicted = None
        self.predicted_goal_pose = None
        self.predicted_trajectory = None
        self.max_traj_dist = 30.0
        
        self.min_diff_for_update = 0.1 **3

        self.goal_thresold = 15.0
        self.next_path_threshold = 10.0
        ## S-crank
        self.arrival_threthold = 1.5
        self.curve_forward_mergin = 10.0
        self.curve_backward_mergin = 6.0

        self.curve_alpha = 0.5
        self.curve_beta = 10.0 / 180 
        self.curve_vel = 0.5
        self.beta_1 = 1.0
        self.beta_2 = 5.0

    ## Check if input data is initialized. ##
    def isReady(self):
        if self.reference_path is None:
            self.get_logger().warning("The reference path data has not ready yet.")
            return False
        if self.current_accel is None:
            self.get_logger().warning("The accel data has not ready yet.")
            return False
        if self.current_odometry is None:
            self.get_logger().warning("The odometry data has not ready yet.")
            return False

        if self.ego_pose is None:
                self.get_logger().warning("The ego pose data has not ready yet.")
                return False
        return True

    ## Callback function for odometry subscriber ##
    def onOdometry(self, msg: Odometry):
        self.current_odometry = msg
        self.ego_pose = self.current_odometry.pose.pose
        self.crrent_vel_x = self.current_odometry.twist.twist.linear.x
        self.crrent_vel_y = self.current_odometry.twist.twist.linear.y
        #self.get_logger().info("odometry {}".format(self.current_odometry.pose))

    ## Callback function for accrel subscriber ##
    def onAcceleration(self, msg: AccelWithCovarianceStamped):
        # return geometry_msgs/Accel 
        self.current_accel = accel = [msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z]

    ## Callback function for predicted objects ##
    def onPerception(self, msg: PredictedObjects):
        self.dynamic_objects = msg

    ## 
    def _get_nearest_path_idx(self, ego_pose, left_bound, right_bound):
        left_diff_x = left_bound[:, 0] - ego_pose[0]
        left_diff_y = left_bound[:, 1] - ego_pose[1]
        left_diff = np.hypot(left_diff_x, left_diff_y)
        right_diff_x = right_bound[:, 0] - ego_pose[0]
        right_diff_y = right_bound[:, 1] - ego_pose[1]
        right_diff = np.hypot(right_diff_x, right_diff_y)
        self.current_left_path_index = left_diff.argmin()
        self.current_right_path_index = right_diff.argmin()
        self.current_path_index = min(self.current_left_path_index,
                                            self.current_right_path_index)
        
        self.next_left_path_index = None
        self.next_right_path_index = None
        for idx in range(len(left_diff)):
            if left_diff[idx] > self.next_path_threshold:
                self.next_left_path_index = idx
                break

        for idx in range(len(right_diff)):
            if right_diff[idx] > self.next_path_threshold:
                self.next_right_path_index = idx
                break

        if self.next_left_path_index is None or self.next_left_path_index == 0:
            self.next_left_path_index = self.current_left_path_index + 1

        if self.next_right_path_index is None or self.next_right_path_index == 0:
            self.next_right_path_index = self.current_right_path_index + 1

    ## Callback function for path subscriber ##
    def onTrigger(self, msg: Path):
        self.reference_path = msg

        ## If sensor data has not prepared yet, publish reference path.
        if not self.isReady():
            self.get_logger().info("Not ready")
            self.pub_path_.publish(self.reference_path)
            return 


        ## Set vehicle status
        if self.vehicle_state == "drive" or self.vehicle_state == "initial":
            if self.crrent_vel_x > 0:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
            else:
                self.stop_time += 0.5
                self.get_logger().info("Stop time {}".format(self.stop_time))

        if self.vehicle_state == "long_stop":
            if self.crrent_vel_x > 0:
                self.vehicle_state = "drive"
                self.stop_time = 0.0

        if self.stop_time > self.stop_duration and self.vehicle_state == "drive":
            self.vehicle_state = "long_stop"

        if self.stop_time > self.stop_duration and (self.vehicle_state == "S-crank-right" or self.vehicle_state == "S-crank-left"):
            self.vehicle_state = "drive"

        ## If find dynamic objects, get object pose
        obj_pose = None
        if self.dynamic_objects is not None:
            obj_info = PredictedObjectsInfo (self.dynamic_objects.objects)
            obj_pose = obj_info.objects_rectangle


        ## Set left and right bound
        self.left_bound = ConvertPointSeq2Array(self.reference_path.left_bound)
        self.right_bound = ConvertPointSeq2Array(self.reference_path.right_bound)


        ## Initialize current path index
        ego_pose_array = ConvertPoint2List(self.ego_pose)
        self._get_nearest_path_idx(ego_pose_array, self.left_bound, self.right_bound)


        ## Visualize objects, vehicle and path on matplotlib
        if self.animation_flag:
            reference_path_array = ConvertPath2Array(self.reference_path)
            self.plot_marker.plot_status(ego_pose_array, 
                                        object_pose =obj_pose, 
                                        left_bound=self.left_bound,
                                        right_bound=self.right_bound,
                                        path_index_left=self.current_left_path_index,
                                        path_index_next_left=self.next_left_path_index,
                                        path_index_right=self.current_right_path_index,
                                        path_index_next_right=self.next_right_path_index,
                                        path=reference_path_array,
                                        predicted_goal_pose=self.predicted_goal_pose,
                                        predicted_trajectory=self.predicted_trajectory,
                                        curve_plot=self.curve_plot,
                                        curve_forward_point=self.curve_forward_point,
                                        curve_backward_point=self.curve_backward_point,
                                        )
        
        ## Check path angle
        if self.current_left_path_index + 2 < len(self.left_bound)\
            and self.current_right_path_index + 2 < len(self.right_bound) :
            next_left_path = calcDistancePoits(self.left_bound[self.current_left_path_index + 1][0:2],
                                                ego_pose_array[0:2])

            next_right_path = calcDistancePoits(self.right_bound[self.current_right_path_index + 1][0:2],
                                                ego_pose_array[0:2])
            
            if next_right_path < self.next_path_threshold or next_left_path < self.next_path_threshold:
                left_bound_1 = self.left_bound[self.current_left_path_index] - self.left_bound[self.current_left_path_index + 1]
                left_bound_2 = self.left_bound[self.current_left_path_index + 2] - self.left_bound[self.current_left_path_index + 1]
                left_bound_1_length = np.hypot(left_bound_1[0], left_bound_1[1])
                left_bound_2_length = np.hypot(left_bound_2[0], left_bound_2[1])

                right_bound_1 = self.right_bound[self.current_right_path_index] - self.right_bound[self.current_right_path_index + 1]
                right_bound_2 = self.right_bound[self.current_right_path_index + 2] - self.right_bound[self.current_right_path_index + 1]

                right_bound_1_length = np.hypot(right_bound_1[0], right_bound_1[1])
                right_bound_2_length = np.hypot(right_bound_2[0], right_bound_2[1])

                cos_left = np.dot(left_bound_1, left_bound_2) / (left_bound_1_length * left_bound_2_length)
                cos_right = np.dot(right_bound_1, right_bound_2) / (right_bound_1_length * right_bound_2_length)
                self.get_logger().info("Left bound cos {}".format(cos_left))
                self.get_logger().info("Right bound cos {}".format(cos_right))

                if self.vehicle_state == "drive":
                    if(cos_left < 0.2 and cos_left > -0.2):
                        if left_bound_1_length > 5:
                            self.vehicle_state = "S-crank-left"
                        else:
                            self.get_logger().info("Left bound length {}".format(left_bound_1_length))
                    elif (cos_right < 0.2 and cos_right > -0.2):
                        if right_bound_1_length > 5:
                            self.vehicle_state = "S-crank-right"
                        else:
                            self.get_logger().info("Right bound length {}".format(right_bound_1_length))

        ## Print vehicle status
        self.get_logger().info("Vehicle state is {}".format(self.vehicle_state))


        ## =====================================================================
        ## ======= Check vehicle status and  publish path or trajectory ========
        ## =====================================================================

        ## If the vehicke is driving or initializing, not execute optimize. ##
        if self.vehicle_state == "drive" or self.vehicle_state == "initial":
            self.get_logger().info("Publish reference path")
            if self.debug:
                self.get_logger().info("Publish reference path")
            self.pub_path_.publish(self.reference_path)
            return

        ## If the vehicle is in S-crank , optimize path ##
        elif self.vehicle_state == "S-crank-right" or self.vehicle_state == "S-crank-left":
            self.optimize_path_for_crank(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
            return 

        ## If the vehicke is stopped long time, optimize trajectory ##
        elif self.vehicle_state == "long_stop":
            self.optimize_path_for_avoidance(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
            self.stop_time = 0.0
            return 
        
        ## If the vehicle is planning, check planning time.##
        elif self.vehicle_state == "long_stop_planning":
            if self.debug:
                self.get_logger().info("Planning now")
            waite_time = (self.get_clock().now().nanoseconds - self.before_exec_time) * (NANO_SECONDS)
            if waite_time < self.duration:
                self.get_logger().info("Remaining wait time {}".format(self.duration - waite_time))
                self.optimize_path_for_avoidance(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
                return
            else:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
                return
            
        elif self.vehicle_state == "crank_planning":
            d = calcDistancePoits(self.predicted_goal_pose[0:2], ego_pose_array[0:2])
            self.get_logger().info("Distance between ego pose and goal {}".format(d))
            self.stop_time = 0.0
            if d < self.arrival_threthold:
                self.vehicle_state = "drive"
                return
            else:
                self.pub_path_.publish(self.planning_path_pub)
                return
        else:
            return 


    def optimize_path_for_crank(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):
        self.get_logger().info("[S-crank]: Calc for {}".format(self.vehicle_state))
        new_path = reference_path
        reference_path_array = ConvertPath2Array(reference_path)
        
        if self.vehicle_state == "S-crank-right":
            target_bound = right_bound
            next_path_index = self.current_right_path_index + 1
            curve_sign = -1
        elif self.vehicle_state ==  "S-crank-left":
            target_bound = left_bound
            next_path_index = self.current_left_path_index + 1
            curve_sign = 1
        else:
            self.get_logger().error("[S-crank]: This optimizer can'nt be used for {}".format(self.vehicle_state))
            

        forward_vec = target_bound[next_path_index + 1] - target_bound[next_path_index]
        forward_vec_length = np.hypot(forward_vec[0], forward_vec[1])
        forward_vec_norm = forward_vec / forward_vec_length 

        backward_vec = target_bound[next_path_index - 1] - target_bound[next_path_index]
        backward_veclength = np.hypot(backward_vec[0], backward_vec[1])
        backward_vec_norm = backward_vec / backward_veclength

        curve_forward_point = target_bound[next_path_index] + self.curve_forward_mergin * forward_vec_norm
        curve_backward_point = target_bound[next_path_index] + self.curve_backward_mergin * backward_vec_norm
        
        ## For debug
        self.curve_forward_point = curve_forward_point
        self.curve_backward_point = curve_backward_point
        
        dist_to_right_angle = target_bound[next_path_index]  - reference_path_array[: , 0:2]
        dist_to_right_angle = np.hypot(dist_to_right_angle[:, 0], dist_to_right_angle[:, 1])

        dist_to_curve_start = curve_backward_point - reference_path_array[: , 0:2]
        dist_to_curve_start= np.hypot(dist_to_curve_start[:, 0], dist_to_curve_start[:, 1])

        dist_to_curve_end =  curve_forward_point - reference_path_array[: , 0:2]
        dist_to_curve_end = np.hypot(dist_to_curve_end[:, 0], dist_to_curve_end[:, 1])

        curve_start_point_idx = dist_to_curve_start.argmin()
        curve_end_point_idx = dist_to_curve_end.argmin()
        
        self.get_logger().info("[S-crank]: Curve start_ponint_index :{} ,Next point index {}".format(curve_start_point_idx,
                                                                                      curve_end_point_idx
                                                                                      ))
        
        
        middle_point_idx = dist_to_right_angle.argmin()
        #middle_point_idx = int((curve_start_point_idx + curve_end_point_idx) / 2 )

        shift_first = curve_backward_point - reference_path_array[curve_start_point_idx , 0:2]


        ## Calculate new path on curve
        rad = 0
        d_rad =  (math.pi) / abs( middle_point_idx - curve_start_point_idx)
        yaw = getYawFromQuaternion(new_path.points[curve_start_point_idx].pose.orientation)
        for idx in range(curve_start_point_idx ,  middle_point_idx):
            reference_path_array[idx][0:2] += shift_first * self.curve_alpha * np.sin(rad)
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel
            if d_rad < math.pi * 0.5:
                yaw += d_rad * self.beta_1
            else:
                yaw -= d_rad * self.beta_2
            new_path.points[idx].pose.orientation = getQuaternionFromEuler(yaw=yaw)
            rad += d_rad

        ## Connect new path and reference path
        connect_vec = reference_path_array[curve_end_point_idx][0:2] - reference_path_array[middle_point_idx -1][0:2]
        connect_vec = connect_vec[0:2] / (curve_end_point_idx - middle_point_idx)
        for idx in range(middle_point_idx, curve_end_point_idx):
            reference_path_array[idx][0:2] = reference_path_array[idx -1 ][0:2] + connect_vec
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel
   

        ## Calculate finish curve
        quarter_point_idx = int((middle_point_idx + curve_end_point_idx)/ 2)
        rad = 0
        #d_rad =  (math.pi) / abs(middle_point_idx - curve_start_point_idx)
        #yaw = getYawFromQuaternion(new_path.points[curve_start_point_idx].pose.orientation)
        for idx in range(quarter_point_idx,  middle_point_idx):
            reference_path_array[idx][0:2] += backward_vec_norm
            new_path.points[idx].pose.position.x = reference_path_array[idx][0]
            new_path.points[idx].pose.position.y = reference_path_array[idx][1]
            new_path.points[idx].longitudinal_velocity_mps = self.curve_vel

        self.predicted_goal_pose = reference_path_array[curve_end_point_idx][0:2]
        

        ## smooting path
        min_path_length = 0.001
        for idx in range(curve_start_point_idx,  curve_end_point_idx):
            p1 = np.array([new_path.points[idx].pose.position.x, new_path.points[idx].pose.position.y])
            p2 = np.array([new_path.points[idx - 1].pose.position.x, new_path.points[idx - 1].pose.position.y])
            dt = calcDistancePoits(p1, p2)
            if dt < min_path_length:
                del new_path.points[idx]

        goal_distance = calcDistancePoits(self.predicted_goal_pose[0:2], reference_path_array[curve_start_point_idx][0:2])
        
        #if goal_distance > self.goal_thresold:
        #    self.vehicle_state = "drive"
        #    self.pub_path_.publish(reference_path)
        #else:

        ## Publish new path
        self.vehicle_state = "crank_planning"
        self.curve_plot = reference_path_array[curve_start_point_idx:curve_end_point_idx + 1]
        
        self.planning_path_pub = new_path
        self.pub_path_.publish(new_path)
        return 

    ## Optimize Path for avoidance
    def optimize_path_for_avoidance(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):
        new_path = reference_path

        # Get the nearest path point.
        reference_path_array = ConvertPath2Array(new_path)
        dist = reference_path_array[: , 0:2] - ego_pose_array[0:2]
        dist = np.hypot(dist[0], dist[1])
        nearest_idx = dist.argmin()

        ## If vehicle is not stopped, publish reference path ##
        points = new_path.points
        self.get_logger().info("Publish optimized path")

        # Set goal pose
        if self.vehicle_state != "long_stop_planning":
            self.predicted_goal_pose = (self.left_bound[self.current_left_path_index+1][0:2] + self.right_bound[self.current_right_path_index+1][0:2]) /2

        ## Set ego_pose_predicted [x value, y value, yaw, vel, yaw vel]
        if self.ego_pose_predicted is None:
            self.get_logger().info("Initialize the ego pose {}".format(np.rad2deg(ego_pose_array)))
            self.ego_pose_predicted = np.array([ego_pose_array[0],
                                                ego_pose_array[1], 
                                                ego_pose_array[2],
                                                0.0,
                                                0.0,
                                                ])
        elif (abs(self.ego_pose_predicted[0] -ego_pose_array[0])< self.min_diff_for_update) or \
            (abs(self.ego_pose_predicted[1] - ego_pose_array[1])  < self.min_diff_for_update):
            self.get_logger().info("The ego pose has not updated yet...")
            self.pub_traj_.publish(self.output_traj)

        else:
            self.get_logger().info("Update the ego pose {}".format(ego_pose_array))
            self.get_logger().info("goal pose {}".format(self.predicted_goal_pose))
            self.ego_pose_predicted[0] = ego_pose_array[0]
            self.ego_pose_predicted[1] = ego_pose_array[1]
            self.ego_pose_predicted[2] = ego_pose_array[2]

        ## Predict new trajectory
        if self.use_dwa:
            self.get_logger().info("Predict ego_pose {}".format(self.ego_pose_predicted))
            u, predicted_traj = self.predictor.get_next_step(self.ego_pose_predicted, 
                                                         self.predicted_goal_pose, 
                                                         object_pose, 
                                                         left_bound[self.current_left_path_index: self.current_left_path_index + 2], 
                                                         right_bound[self.current_right_path_index: self.current_right_path_index + 2])

            if len(predicted_traj) == 0:
                return

            self.ego_pose_predicted[3] = u[0]
            self.ego_pose_predicted[4] = u[1]
            output_traj = Trajectory()
            output_traj.points = convertPathToTrajectoryPoints(self.reference_path, len(predicted_traj) + 1)
            output_traj.header = new_path.header
            output_traj.header.stamp = self.get_clock().now().to_msg()

            point_dist = 0.0
            calc_point = self.ego_pose_predicted[0:2]
            threshold = 2.0
            ignore_point = 0
            output_traj.points[0].pose.position.x = ego_pose_array[0]
            output_traj.points[0].pose.position.y = ego_pose_array[1]
            output_traj.points[0].longitudinal_velocity_mps = 0.5
            for idx in range(len(predicted_traj) -1):
                dt = calcDistancePoits(predicted_traj[idx][0:2], calc_point[0:2])
                point_dist += dt
                if point_dist > threshold:
                    output_traj.points[idx + 1].pose.position.x = predicted_traj[idx][0]
                    output_traj.points[idx + 1].pose.position.y = predicted_traj[idx][1]
                    output_traj.points[idx].pose.orientation = getQuaternionFromEuler(yaw=predicted_traj[idx][2])
                    output_traj.points[idx + 1].longitudinal_velocity_mps = 0.5
                    calc_point = predicted_traj[idx][0:2]
                    point_dist = 0.0
                else:
                    ignore_point += 1

            for _ in range(ignore_point):
                output_traj.points.pop(-1)

            if self.animation_flag:
                self.get_logger().info("predicted trajectory points {}".format(len(predicted_traj)))
                self.predicted_trajectory = predicted_traj

        ## Use reference path as trajectory
        else:
            traj_dist = 0.0
            output_traj = Trajectory()
            output_traj.header = new_path.header
            output_traj.header.stamp = self.get_clock().now().to_msg()
            output_traj.points = convertPathToTrajectoryPoints(self.reference_path, len(self.reference_path.points))
            for idx in range(1, len(reference_path_array)):
                traj_dist  += calcDistancePoits(reference_path_array[idx][0:2], reference_path_array[idx - 1][0:2])
                if traj_dist > self.max_traj_dist:
                    break
            ignore_point = len(reference_path_array) - idx
            for _ in range(ignore_point):
                if len(output_traj.points) <= 10:
                    break
                output_traj.points.pop(-1)
        
        self.output_traj = output_traj
        self.get_logger().info("Output trajectory points {}".format(len(output_traj.points)))

        ## Path Optimize ##
        if self.vehicle_state != "long_stop_planning":
            self.before_exec_time = self.get_clock().now().nanoseconds
            self.vehicle_state = "long_stop_planning"

        
        ## Publish traj
        self.planning_traj_pub = self.output_traj
        self.pub_traj_.publish(self.output_traj)

def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()