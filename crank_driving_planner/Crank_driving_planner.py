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
        print(time)

        # Initialize input ##
        self.reference_path = None
        self.current_accel = None
        self.current_odometry = None
        self.crrent_longitudinal_velocity = 0.0
        self.ego_pose = None
        self.dynamic_objects = None
        self.left_bound  = None
        self.right_bound  = None
        self.curve_plot = None

        self.vehicle_state = "initial"
        self.before_exec_time = -9999 * NANO_SECONDS
        self.duration = 10.0

        ## Check stop time
        self.stop_time = 0.0
        self.stop_duration = 75

        self.current_path_index = None
        self.next_path_index = None
        self.next_path_threshold = 3.0

        self.animation_flag = True
        self.debug = False
        
        if self.animation_flag:
            self.plot_marker = PlotMarker()

        ## Dynamic Window Approach
        self.dwa_config = Config()
        self.predictor = DynamicWindowApproach(self.dwa_config)
        self.ego_pose_predicted = None
        self.predicted_goal_pose = None
        self.predicted_trajectory = None
        self.min_diff_for_update = 0.1 **3


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
        self.current_path_index = min(left_diff.argmin(), right_diff.argmin())
        self.next_path_index = self.current_path_index + 1 

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
                                        index_min=self.current_path_index,
                                        index_max=self.current_path_index + 1,
                                        path=reference_path_array,
                                        predicted_goal_pose=self.predicted_goal_pose,
                                        predicted_trajectory=self.predicted_trajectory,
                                        curve_plot=self.curve_plot
                                        )
        
        ## Check path angle
        if self.next_path_index + 1 < len(self.left_bound)\
            and self.next_path_index + 1 < len(self.right_bound) :
            next_left_path = calcDistancePoits(self.left_bound[self.next_path_index][0:2],
                                                ego_pose_array[0:2])

            next_right_path = calcDistancePoits(self.right_bound[self.next_path_index][0:2],
                                                ego_pose_array[0:2])
            
            if next_right_path < self.next_path_threshold or next_left_path < self.next_path_threshold:
                left_bound_1 = self.left_bound[self.next_path_index - 1] - self.left_bound[self.next_path_index]
                left_bound_2 = self.left_bound[self.next_path_index + 1] - self.left_bound[self.next_path_index]
                left_bound_1_length = np.hypot(left_bound_1[0], left_bound_1[1])
                left_bound_2_length = np.hypot(left_bound_2[0], left_bound_2[1])

                right_bound_1 = self.right_bound[self.next_path_index - 1] - self.right_bound[self.next_path_index]
                right_bound_2 = self.right_bound[self.next_path_index + 1] - self.right_bound[self.next_path_index]

                right_bound_1_length = np.hypot(right_bound_1[0], right_bound_1[1])
                right_bound_2_length = np.hypot(right_bound_2[0], right_bound_2[1])

                cos_left = np.dot(left_bound_1, left_bound_2) / (left_bound_1_length * left_bound_2_length)
                cos_right = np.dot(right_bound_1, right_bound_2) / (right_bound_1_length * right_bound_2_length)
                self.get_logger().info("Left bound cos {}".format(cos_left))
                self.get_logger().info("right bound cos {}".format(cos_right))

                if self.vehicle_state == "drive":
                    if (cos_left > -0.75 and cos_left < -0.2) or (cos_left > 0.2 and cos_left < 0.75):
                        self.vehicle_state = "S-crank-left"
                    elif (cos_right > -0.75 and cos_right < -0.2) or (cos_right > 0.2 and cos_right < 0.75):
                        self.vehicle_state = "S-crank-right"

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
            return 
        
        ## If the vehicle is planning, check planning time.##
        elif self.vehicle_state == "long_stop_planning":
            if self.debug:
                self.get_logger().info("Planning now")
            waite_time = (self.get_clock().now().nanoseconds - self.before_exec_time) * (NANO_SECONDS)
            if waite_time < self.duration:
                self.get_logger().info("Remaining wait time {}".format(self.duration - waite_time))
                self.optimize_path_for_avoidance(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
            else:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
                return
        elif self.vehicle_state == "crank_planning":
            arrival_threthold = 0.2
            d = calcDistancePoits(self.predicted_goal_pose[0:2], ego_pose_array[0:2])
            self.get_logger().info("Distance between ego pose and goal {}".format(d))
        else:
            return 


    def optimize_path_for_crank(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):
        self.get_logger().info("Calc for {}".format(self.vehicle_state))
        new_path = reference_path
        reference_path_array = ConvertPath2Array(reference_path)
        
        if self.vehicle_state == "S-crank-right":
            target_bound = right_bound
        elif self.vehicle_state ==  "S-crank-left":
            target_bound = left_bound
        else:
            self.get_logger().error("This optimizer can'nt be used for {}".format(self.vehicle_state))
            

        dist_bound_point_current = reference_path_array[: , 0:2] - target_bound[self.next_path_index]
        dist_bound_point_current = np.hypot(dist_bound_point_current[:, 0], dist_bound_point_current[:, 1])

        dist_bound_point_next= reference_path_array[: , 0:2] - target_bound[self.next_path_index + 1]
        dist_bound_point_next = np.hypot(dist_bound_point_next[:, 0], dist_bound_point_next[:, 1])

        nearest_cuurent_point_idx = dist_bound_point_current.argmin()
        nearest_next_point_idx = dist_bound_point_next.argmin()
        
        self.get_logger().info("Current_ponint_index :{} ,Next point index {}".format(nearest_cuurent_point_idx,
                                                                                      nearest_next_point_idx
                                                                                      ))
        
        middle_point_idx = int((nearest_cuurent_point_idx + nearest_next_point_idx) / 2 )

        shift_first = reference_path_array[nearest_cuurent_point_idx , 0:2] - target_bound[self.next_path_index]
        shift_second = reference_path_array[nearest_next_point_idx , 0:2] - target_bound[self.next_path_index + 1]
        rad = 0
        d_rad =  (math.pi * 1/2) / abs(middle_point_idx - nearest_cuurent_point_idx)
        alpha = 0.8
        for idx in range(nearest_cuurent_point_idx, middle_point_idx):
            reference_path_array[idx][0:2] += shift_first * alpha * np.sin(rad)
            rad += d_rad

        connect_vec = reference_path_array[nearest_next_point_idx][0:2] - reference_path_array[middle_point_idx -1][0:2]
        connect_vec = connect_vec[0:2] / (nearest_next_point_idx - middle_point_idx)
        #rad = 0
        #d_rad =  (math.pi * 1/2) / abs(nearest_next_point_idx - middle_point_idx)
        for idx in range(middle_point_idx, nearest_next_point_idx):
            reference_path_array[idx][0:2] = reference_path_array[idx -1 ][0:2] + connect_vec
   
        self.predicted_goal_pose = reference_path_array[nearest_next_point_idx][0:2]
        
        #if calcDistancePoits(reference_path_array[nearest_next_point_idx , 0:2], ego_pose_array[0:2]) < arrival_threthold:
        self.vehicle_state = "crank_planning"
        self.curve_plot = reference_path_array[nearest_cuurent_point_idx:nearest_next_point_idx + 1]
        self.pub_path_.publish(new_path)

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
            self.predicted_goal_pose = (self.left_bound[self.current_path_index+1][0:2] + self.right_bound[self.current_path_index+1][0:2]) /2

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
        self.get_logger().info("Predict ego_pose {}".format(self.ego_pose_predicted))
        u, predicted_traj = self.predictor.get_next_step(self.ego_pose_predicted, 
                                                         self.predicted_goal_pose, 
                                                         object_pose, 
                                                         left_bound[self.current_path_index: self.current_path_index + 2], 
                                                         right_bound[self.current_path_index: self.current_path_index + 2])

        if len(predicted_traj) == 0:
            return

        self.ego_pose_predicted[3] = u[0]
        self.ego_pose_predicted[4] = u[1]

        ## Generate trajectory
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
        for idx in reversed(range(len(predicted_traj) - 1)):
            dt = calcDistancePoits(predicted_traj[idx][0:2], calc_point[0:2])
            point_dist += dt
            if point_dist > threshold:
                output_traj.points[idx + 1].pose.position.x = predicted_traj[idx][0]
                output_traj.points[idx + 1].pose.position.y = predicted_traj[idx][1]
                #output_traj.points[idx].pose.orientation = getQuaternionFromEuler(yaw=predicted_traj[idx][2])
                output_traj.points[idx + 1].longitudinal_velocity_mps = 0.5
                point_dist = 0.0
            else:
                ignore_point += 1

        for k in range(ignore_point):
            output_traj.points.pop(-1)

        self.output_traj = output_traj
        self.get_logger().info("Output trajectory points {}".format(len(output_traj.points)))

        ## Path Optimize ##
        if self.vehicle_state != "long_stop_planning":
            self.before_exec_time = self.get_clock().now().nanoseconds
            self.vehicle_state = "long_stop_planning"

        if self.animation_flag:
            self.get_logger().info("predicted trajectory points {}".format(len(predicted_traj)))
            self.predicted_trajectory = predicted_traj
        
        ## Publish traj
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