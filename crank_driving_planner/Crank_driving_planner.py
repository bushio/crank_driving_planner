import time
import rclpy
import numpy as np
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory, Path
from geometry_msgs.msg import AccelWithCovarianceStamped, Point
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry

from .util import *
from .bound_checker import *
from .predicted_objects_info import PredictedObjectsInfo

## For plot
from .debug_plot import PlotMarker

from .curve_generator import CurveGenerator
from .predict_path_generator import PathPredictor
from .config import CurveConfig

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
        self.pub_traj_ = self.create_publisher(Trajectory, "~/output/trajectory", 10)


        ## Road config

        self.curve_cfg = CurveConfig()

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
        self.predicted_duration = 10.0

        self.planning_path_pub = None

        ## Check stop time
        self.stop_time = 0.0
        self.stop_duration = 60

        self.animation_flag = self.curve_cfg.animation_flag
        self.debug = False
        
        if self.animation_flag:
            self.plot_marker = PlotMarker()
            self.curve_plot = None
            self.curve_backward_point = None
            self.curve_forward_point = None
            self.debug_point = None

        self.ego_pose_predicted = None
        self.predicted_goal_pose = None
        self.predicted_trajectory = None
        
        self.max_traj_dist = 15.0
        self.next_path_threshold = 10.0
        self.arrival_threthold = self.curve_cfg.arrival_threthold

        self.use_dwa = False
        if self.use_dwa:
            self.dwa_predictor = PathPredictor(self.get_logger(), self.get_clock())
        self.curve_generator = CurveGenerator(self.get_logger())

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

    ## Callback function for path subscriber ##
    def onTrigger(self, msg: Path):
        self.reference_path = msg

        ## If sensor data has not prepared yet, publish reference path.
        if not self.isReady():
            self.get_logger().info("Not ready")
            self.pub_path_.publish(self.reference_path)
            return 


        ## Set vehicle status
        if self.vehicle_state == "drive":
            if self.crrent_vel_x > 0:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
            else:
                self.stop_time += 0.5
                self.get_logger().info("Stop time {}".format(self.stop_time))
        elif self.vehicle_state == "initial":
            if self.crrent_vel_x > 0:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
        else:
            if self.crrent_vel_x > 0:
                self.stop_time = 0.0
            else:
                self.stop_time += 0.5
                self.get_logger().info("Stop time {}".format(self.stop_time))

        if self.vehicle_state == "long_stop":
            if self.crrent_vel_x > 0:
                self.vehicle_state = "drive"
                self.stop_time = 0.0

        if self.stop_time > self.stop_duration:
            if self.vehicle_state == "drive":
                self.vehicle_state = "long_stop"
                self.stop_time = 0.0
            elif self.vehicle_state == "S-crank-right" or self.vehicle_state == "S-crank-left":
                self.vehicle_state = "drive"
                self.stop_time = 0.0
            elif self.vehicle_state =="crank_planning":
                self.vehicle_state = "drive"
                self.stop_time = 0.0

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
        self.current_left_path_index ,self.next_left_path_index = \
            get_nearest_path_idx(ego_pose_array, self.left_bound, next_path_threshold=5.0)

        self.current_right_path_index ,self.next_right_path_index = \
            get_nearest_path_idx(ego_pose_array, self.right_bound, next_path_threshold=5.0)

        reference_path_array = ConvertPath2Array(self.reference_path)
        ## Visualize objects, vehicle and path on matplotlib
        if self.animation_flag:
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
                                        vis_point=self.debug_point
                                        )
        
        ## Check path angle
        if self.current_left_path_index + 2 < len(self.left_bound)\
            and self.current_right_path_index + 2 < len(self.right_bound) :
            next_left_path = calcDistancePoits(self.left_bound[self.current_left_path_index + 1][0:2],
                                                ego_pose_array[0:2])

            next_right_path = calcDistancePoits(self.right_bound[self.current_right_path_index + 1][0:2],
                                                ego_pose_array[0:2])
            
            if next_right_path < self.next_path_threshold or next_left_path < self.next_path_threshold:
                if self.current_left_path_index > 0:
                    cos_left  = getCosFromLines(self.left_bound[self.current_left_path_index - 1], 
                                            self.left_bound[self.current_left_path_index],
                                            self.left_bound[self.current_left_path_index + 1])
                else:
                    cos_left = -999

                if self.current_right_path_index > 0:
                    cos_right = getCosFromLines(self.right_bound[self.current_right_path_index - 1],
                                            self.right_bound[self.current_right_path_index],
                                            self.right_bound[self.current_right_path_index + 1])
                else:
                    cos_right = -1
                
                cos_left_next = getCosFromLines(self.left_bound[self.current_left_path_index], 
                                           self.left_bound[self.current_left_path_index + 1],
                                           self.left_bound[self.current_left_path_index + 2])
                
                cos_right_next = getCosFromLines(self.right_bound[self.current_right_path_index],
                                            self.right_bound[self.current_right_path_index + 1],
                                            self.right_bound[self.current_right_path_index + 2])
                self.get_logger().info("Left bound cos current {} next {}".format(cos_left, cos_left_next))
                self.get_logger().info("Right bound cos current {} next {}".format(cos_right, cos_right_next))

                #if self.vehicle_state == "drive":
                if self.vehicle_state == "drive":
                    if(cos_left_next < 0.2 and  cos_left_next> -0.2) or (cos_left < 0.2 and  cos_left > -0.2):
                        self.vehicle_state = "S-crank-right"

                    elif (cos_right_next < 0.2 and cos_right_next > -0.2) or  (cos_right < 0.2 and cos_right > -0.2):
                        self.vehicle_state = "S-crank-left"

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


            self.obstacle_check_on_path(reference_path_array, ego_pose_array, obj_pose)

            self.pub_path_.publish(self.reference_path)
            return

        ## If the vehicle is in S-crank , optimize path ##
        elif self.vehicle_state == "S-crank-right" or self.vehicle_state == "S-crank-left":
            new_path = self.optimize_path_for_crank(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
            self.pub_path_.publish(new_path)
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
            if waite_time < self.predicted_duration:
                self.get_logger().info("Remaining wait time {}".format(self.predicted_duration - waite_time))
                self.optimize_path_for_avoidance(self.reference_path, ego_pose_array, obj_pose, self.left_bound, self.right_bound)
                return
            else:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
                return
        
        ## If the vehicle is crank_planning, check distance from th predicted goal pose##
        elif self.vehicle_state == "crank_planning":
            d = calcDistancePoits(self.predicted_goal_pose[0:2], ego_pose_array[0:2])
            self.get_logger().info("Distance between ego pose and goal {}".format(d))
            if d < self.arrival_threthold:
                self.vehicle_state = "drive"
                self.stop_time = 0.0
                self.curve_generator.reset_enable_planning()
                return
            else:
                self.pub_path_.publish(self.planning_path_pub)
                return
        else:
            return 

    def optimize_path_for_crank(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):
        self.get_logger().info("[S-crank]: Calc for {}".format(self.vehicle_state))
        if self.vehicle_state == "S-crank-right":
            outer_bound= left_bound
            inner_bound = right_bound
            curve_sign = -1
        elif self.vehicle_state ==  "S-crank-left":
            outer_bound = right_bound
            inner_bound= left_bound
            curve_sign = 1
        else:
            self.get_logger().error("[S-crank]: This optimizer can'nt be used for {}".format(self.vehicle_state))
        
        new_path = reference_path
        reference_path_array = ConvertPath2Array(reference_path)

        ## Get the diag line of inner bound.
        diag_idx = get_diag_point(inner_bound)

        ## Get the sharp point of outer bound.
        sharp_index = get_sharp_point(outer_bound)
        if (sharp_index is None) or (diag_idx is None):
            return new_path
        
        ## Judge actions from Road width 
        road_width = getRoadWidth(inner_bound, outer_bound, diag_idx, sharp_index)
        self.get_logger().info("Road width {}".format(road_width))
        if road_width >= 3.2:
            R = self.curve_cfg.circle_radius[0]
            curve_angle = self.curve_cfg.circle_angle_rate[0]
            predict_curve = self.curve_cfg.exec_predict[0]
            inner_start_mergin = self.curve_cfg.inner_start_mergin[0]
            inner_finish_mergin = self.curve_cfg.inner_finish_mergin[0]
        elif road_width >= 2.5 and road_width < 3.2:
            R = self.curve_cfg.circle_radius[1]
            curve_angle = self.curve_cfg.circle_angle_rate[1]
            predict_curve = self.curve_cfg.exec_predict[1]
            inner_start_mergin = self.curve_cfg.inner_start_mergin[1]
            inner_finish_mergin = self.curve_cfg.inner_finish_mergin[1]
        elif road_width > 2.0  and road_width < 2.5:
            R = self.curve_cfg.circle_radius[2]
            curve_angle = self.curve_cfg.circle_angle_rate[2]
            predict_curve = self.curve_cfg.exec_predict[2]
            inner_start_mergin = self.curve_cfg.inner_start_mergin[2]
            inner_finish_mergin = self.curve_cfg.inner_finish_mergin[2]
        else:
            R = self.curve_cfg.circle_radius[3]
            curve_angle = self.curve_cfg.circle_angle_rate[3]
            predict_curve = self.curve_cfg.exec_predict[3]
            inner_start_mergin = self.curve_cfg.inner_start_mergin[3]
            inner_finish_mergin = self.curve_cfg.inner_finish_mergin[3]
        
        dist_to_sharp_point = calcDistancePoits(ego_pose_array[0:2], outer_bound[sharp_index])
        
        if dist_to_sharp_point > self.curve_cfg.sharp_dist_threshold:
            self.get_logger().info("Distance to_sharp_point {}".format(dist_to_sharp_point))
            predict_curve = False

        ## Predict curve path
        if predict_curve:
            self.get_logger().info("Predict curve")
            result = self.curve_generator.generate_curve_circle(
                new_path, 
                reference_path_array, 
                outer_bound,
                inner_bound,
                diag_idx,
                sharp_index,
                road_width,
                curve_sign,
                carve_radius = R,
                curve_angle = curve_angle,
                inner_start_mergin=inner_start_mergin,
                inner_finish_mergin=inner_finish_mergin
                )
        else:
            result = None
    
        if result is not None:
            self.predicted_goal_pose = self.curve_generator.predicted_goal_pose
            self.curve_plot = self.curve_generator.curve_plot
            self.debug_point = self.curve_generator.debug_point
            self.vehicle_state = "crank_planning"
            new_path = result

        self.planning_path_pub = new_path
        return new_path

    ## Optimize Path for avoidance
    def optimize_path_for_avoidance(self, reference_path, ego_pose_array, object_pose, left_bound, right_bound):

        ## If vehicle is not stopped, publish reference path ##
        self.get_logger().info("Publish optimized path")

        # Set goal pose
        if self.vehicle_state != "long_stop_planning":
            self.predicted_goal_pose = (self.left_bound[self.current_left_path_index+1][0:2] + self.right_bound[self.current_right_path_index+1][0:2]) /2

        ## Predict new trajectory
        if self.use_dwa:
            reference_path_array = ConvertPath2Array(reference_path)
            output_traj = self.dwa_predictor.predict_path_by_dwa(
                            reference_path, 
                            ego_pose_array, 
                            self.predicted_goal_pose, 
                            object_pose, 
                            left_bound, 
                            right_bound,
                            self.current_left_path_index, 
                            self.current_right_path_index)
            predicted_traj = self.dwa_predictor.predicted_traj
            if self.animation_flag:
                self.get_logger().info("predicted trajectory points {}".format(len(predicted_traj)))
                self.predicted_trajectory = predicted_traj
        
        ## Use reference path as trajectory
        else:
            reference_path_array = ConvertPath2Array(reference_path)
            nearest_idx = getNearestPointIndex(ego_pose_array[0:2], reference_path_array[:, 0:2])
            for i in range(50):
                reference_path_array[nearest_idx + i][1] -= 0.5
                reference_path.points[nearest_idx + i].pose.position.y -= reference_path_array[nearest_idx + i][1]
            
            traj_dist = 0.0
            output_traj = Trajectory()
            output_traj.header = reference_path.header
            output_traj.header.stamp = self.get_clock().now().to_msg()
            output_traj.points = convertPathToTrajectoryPoints(self.reference_path, 
                                                               start_idx=nearest_idx,
                                                               end_idx=len(self.reference_path.points) - 1)
            for idx in range(nearest_idx, len(reference_path_array)):
                traj_dist  += calcDistancePoits(reference_path_array[idx][0:2], reference_path_array[idx - 1][0:2])
                if traj_dist > self.max_traj_dist:
                    break

            ignore_point = len(reference_path_array) - idx
            for _ in range(ignore_point):
                if len(output_traj.points) <= 10:
                    break
                output_traj.points.pop(-1)
            self.predicted_trajectory = reference_path_array
        
        self.output_traj = output_traj
        self.get_logger().info("Output trajectory points {}".format(len(output_traj.points)))

        ## Path Optimize ##
        if self.vehicle_state != "long_stop_planning":
            self.before_exec_time = self.get_clock().now().nanoseconds
            self.vehicle_state = "long_stop_planning"

        ## Publish traj
        self.pub_traj_.publish(self.output_traj)
        
    
    def obstacle_check_on_path(self, reference_path_array, ego_pose_array, object_pose):
        if object_pose is None:
            return
        if len(object_pose) == 0:
            return

        dist = calcDistancePoitsFromArray(ego_pose_array, object_pose)
        nearest_idx = dist.argmin()
        
        obj_thrshold = 10
        if dist[nearest_idx] < obj_thrshold:
            dit_path_ob = calcDistancePoitsFromArray(object_pose[nearest_idx], reference_path_array)
            nearest_idx_path_ob = dit_path_ob.argmin()
            self.debug_point = reference_path_array[nearest_idx_path_ob][0:2]
            #self.get_logger().info("Distance fron objects is {}".format(dit_path_ob[nearest_idx_path_ob]))
        else:
            return

def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()