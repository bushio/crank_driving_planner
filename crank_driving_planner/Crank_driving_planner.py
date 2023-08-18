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

from .config import Config
from .dynamic_window_approach import DynamicWindowApproach

from .debug_plot import PlotMarker

#from motion_utils import calcLongitudinalOffsetPose

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


        # Path publisher. Remap "/planning/scenario_planning/lane_driving/path" ##
        self.pub_path_ = self.create_publisher(Path, 
                                               "~/output/path", 
                                                10)
        # Predicted objects subscriber
        self.create_subscription(PredictedObjects, "~/input/perception", self.onPerception, 10)
        
        # Initialize input ##
        self.reference_path = None
        self.current_accel = None
        self.current_odometry = None
        self.crrent_longitudinal_velocity = 0.0
        self.ego_pose = None
        self.dynamic_objects = None

        self.vehicle_state = "drive"
        self.nano_seconds = 1000**3
        self.before_exec_time = -9999 * self.nano_seconds
        self.duration = 10.0

        self.stop_time = 0.0
        self.stop_duration = 30

        self.plot_marker = PlotMarker()

        

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
        self.crrent_vel = self.current_odometry.twist.twist.linear.x
        #self.get_logger().info("odometry {}".format(self.current_odometry.pose))

        if self.vehicle_state == "planning":
            return 

        if self.crrent_vel > 0:
            self.vehicle_state = "drive"
            self.stop_time = 0.0
        else:
            self.stop_time += 0.1

        if self.stop_time > self.stop_duration:
            self.vehicle_state = "stop"

    ## Callback function for accrel subscriber ##
    def onAcceleration(self, msg: AccelWithCovarianceStamped):
        # return geometry_msgs/Accel 
        self.current_accel = accel = [msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z]

    ## Callback function for predicted objects ##
    def onPerception(self, msg: PredictedObjects):
        self.dynamic_objects = msg

    ## Callback function for path subscriber ##
    def onTrigger(self, msg: Path):
        self.get_logger().info("Get path. Processing crank driving planner...")
        self.get_logger().info("Vehicle state is {}".format(self.vehicle_state))
        self.reference_path = msg
        obj_pose = None
        if self.dynamic_objects is not None:
            self.get_logger().info("Objects num {}".format(len(self.dynamic_objects.objects)))
            obj_info = PredictedObjectsInfo (self.dynamic_objects.objects)
            obj_pose = obj_info.objects_rectangle

        if not self.isReady():
            self.get_logger().info("Not ready")
            self.pub_path_.publish(self.reference_path)
            return 

        
        ego_pose_ = ConvertPoint2List(self.ego_pose)
        
        left_bound = ConvertPointSeq2Array(self.reference_path.left_bound)
        right_bound = ConvertPointSeq2Array(self.reference_path.right_bound)
        self.plot_marker.plot_status(ego_pose_, 
                                    object_pose =obj_pose, 
                                    left_bound=left_bound,
                                    right_bound=right_bound,
                                    )

        ## If the vehicke is driving, not execute optimize. ##
        if self.vehicle_state == "drive":
            self.get_logger().info("Publish reference path")
            self.pub_path_.publish(self.reference_path)
            return

        elif self.vehicle_state == "planning":
            self.get_logger().info("Planning now")
            waite_time = (self.get_clock().now().nanoseconds - self.before_exec_time)/(self.nano_seconds)
            if waite_time < self.duration:
                self.get_logger().info("Remaining wait time {}".format(self.duration - waite_time))
                return 



        #=============
        self.get_logger().info("Publish optimized path")

        ## If vehicle is not stopped, publish reference path ##
        points = self.reference_path.points
        self.get_logger().info("Points num {}".format(len(points)))
        ego_pose_ = ConvertPoint2List(self.ego_pose)
        
        self.get_logger().info("Left bound{}".format(self.reference_path.left_bound))
        ## Path Optimize ##

        self.before_exec_time = self.get_clock().now().nanoseconds
        self.vehicle_state = "planning"
        self.pub_path_.publish(self.reference_path)

def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()