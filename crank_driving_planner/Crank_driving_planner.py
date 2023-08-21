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

        self.vehicle_state = "drive"
        self.before_exec_time = -9999 * NANO_SECONDS
        self.duration = 15.0

        ## Check stop time
        self.stop_time = 0.0
        self.stop_duration = 30

        self.current_path_index =None
        self.path_search_range = 3
        self.change_next_path = 3.0 #[m]

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

        if self.vehicle_state == "planning":
            return 

        if self.crrent_vel_x > 0:
            self.vehicle_state = "drive"
            self.stop_time = 0.0
        else:
            self.stop_time += 0.1

        if self.stop_time > self.stop_duration and self.vehicle_state == "drive":
            self.vehicle_state = "stop"

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

    ## Callback function for path subscriber ##
    def onTrigger(self, msg: Path):
        self.reference_path = msg
        
        ## Print vehicle status
        if self.debug:
            self.get_logger().info("Get path. Processing crank driving planner...")
            self.get_logger().info("Vehicle state is {}".format(self.vehicle_state))


        ## If find dynamic objects, get object pose
        obj_pose = None
        if self.dynamic_objects is not None:
            obj_info = PredictedObjectsInfo (self.dynamic_objects.objects)
            obj_pose = obj_info.objects_rectangle


        ## If sensor data has not prepared yet, publish reference path.
        if not self.isReady():
            self.get_logger().info("Not ready")
            self.pub_path_.publish(self.reference_path)
            return 


        ## Set left and right bound
        self.left_bound = ConvertPointSeq2Array(self.reference_path.left_bound)
        self.right_bound = ConvertPointSeq2Array(self.reference_path.right_bound)


        ## Initialize current path index
        ego_pose_array = ConvertPoint2List(self.ego_pose)
        self._get_nearest_path_idx(ego_pose_array, self.left_bound, self.right_bound)


        ## Check the angle of next path


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
                                        predicted_trajectory=self.predicted_trajectory
                                        )
        
        ## Check path angle
        if self.current_path_index < len(self.left_bound) and self.current_path_index > 0 \
            and self.current_path_index < len(self.left_bound) and self.current_path_index > 0 :

            left_bound_1 = self.left_bound[self.current_path_index - 1] - self.left_bound[self.current_path_index]
            left_bound_2 = self.left_bound[self.current_path_index + 1] - self.left_bound[self.current_path_index]
            left_bound_1_length = np.hypot(left_bound_1[0], left_bound_1[1])
            left_bound_2_length = np.hypot(left_bound_2[0], left_bound_2[1])

            right_bound_1 = self.right_bound[self.current_path_index - 1] - self.right_bound[self.current_path_index]
            right_bound_2 = self.right_bound[self.current_path_index + 1] - self.right_bound[self.current_path_index]

            right_bound_1_length = np.hypot(right_bound_1[0], right_bound_1[1])
            right_bound_2_length = np.hypot(right_bound_2[0], right_bound_2[1])

            cos_left = np.dot(left_bound_1, left_bound_2) / (left_bound_1_length * left_bound_2_length)
            cos_right = np.dot(right_bound_1, right_bound_2) / (right_bound_1_length * right_bound_2_length)
            self.get_logger().info("Left bound cos {}".format(cos_left))
            self.get_logger().info("right bound cos {}".format(cos_right))

            if cos_left > -0.5 or cos_right > -0.5:
                self.vehicle_state = "S-crank"


        ## If the vehicke is driving, not execute optimize. ##
        if self.vehicle_state == "drive":
            self.get_logger().info("Publish reference path")
            if self.debug:
                self.get_logger().info("Publish reference path")
            self.pub_path_.publish(self.reference_path)
            return


        ## If the vehicke is planning, check planning time.##
        elif self.vehicle_state == "planning":
            if self.debug:
                self.get_logger().info("Planning now")
            waite_time = (self.get_clock().now().nanoseconds - self.before_exec_time) * (NANO_SECONDS)
            if waite_time < self.duration:
                self.get_logger().info("Remaining wait time {}".format(self.duration - waite_time))

            else:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
                return

        ## ===================================================
        ## ======= Optimize path  for stop status ============
        ## ===================================================
        self.optimize_path(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)

        return 
    ## Optimize Path for avoidance
    def optimize_path(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):
        new_path = reference_path

        # Get the nearest path point.
        reference_path_array = ConvertPath2Array(new_path)
        dist = reference_path_array[: , 0:2] - ego_pose_array[0:2]
        nearest_idx = np.argmin(dist, axis=0)[0]

        ## If vehicle is not stopped, publish reference path ##
        points = new_path.points
        self.get_logger().info("Publish optimized path")

        # Set goal pose
        if self.vehicle_state != "planning":
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
            return 
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
        output_traj.points = convertPathToTrajectoryPoints(self.reference_path, len(predicted_traj))
        output_traj.header = new_path.header

        for idx in reversed(range(len(predicted_traj))):
            output_traj.points[idx].pose.position.x = predicted_traj[idx][0]
            output_traj.points[idx].pose.position.y = predicted_traj[idx][1]
            output_traj.points[idx].pose.orientation = getQuaternionFromEuler(yaw=predicted_traj[idx][2])
            output_traj.points[idx].longitudinal_velocity_mps = 1.0
            #print(np.linalg.norm(reference_path_array[idx, 0:2] - reference_path_array[idx - 1, 0:2]))


        ## Path Optimize ##
        if self.vehicle_state != "planning":
            self.before_exec_time = self.get_clock().now().nanoseconds
            self.vehicle_state = "planning"

        if self.animation_flag:
            #self.get_logger().info("Nearest points {}".format(output_traj.points[nearest_idx - goal_pose_offset].pose.position))
            self.get_logger().info("predicted trajectory points {}".format(len(predicted_traj)))
            self.predicted_trajectory = predicted_traj
        
        ## Publish traj
        self.pub_traj_.publish(output_traj)
        #self.pub_path_.publish(new_path)

def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()