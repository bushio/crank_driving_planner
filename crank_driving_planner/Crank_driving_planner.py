import time
import rclpy
import numpy as np
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import AccelWithCovarianceStamped, Point
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import MotionState
from .trajectory_uitl import *

#from motion_utils import calcLongitudinalOffsetPose

class CrankDrigingPlanner(Node):
    def __init__(self):
        super().__init__('CrankDrigingPlanner')
        self.get_logger().info("Start CrankDrigingPlanner")
        
        # Reference trajectory subscriber. Remap "obstacle_stop_planner/trajectory"
        self.create_subscription(Trajectory ,"~/input/trajectory", self.onTrigger, 1)

        # Accrel subscriber
        self.create_subscription(AccelWithCovarianceStamped, "~/input/acceleration", self.onAcceleration, 1)

        # Vehicle odometry subscriber
        self.create_subscription(Odometry, "~/input/odometry", self.onOdometry, 1)

        # velocity report subscriber
        self.create_subscription(VelocityReport, "~/input/velocity_report", self.onVelocityReport, 1)

        # motion state subscriber
        self.create_subscription(MotionState, "~/input/motion_state", self.onMotion, 1)


        # Trajectory publisher. Remap "/planning/scenario_planning/lane_driving/trajectory"
        self.pub_trajectory_ = self.create_publisher(Trajectory, 
                                                     "~/output/trajectory", 
                                                     1 )
        
        
        # Initialize input
        self.reference_trj = None
        self.current_accel = None
        self.current_odometry = None
        self.crrent_velocity_report = None
        self.crrent_longitudinal_velocity = 0.0
        self.ego_pose = None
        self.motion_state = None

    # Check if input data is initialized.
    def isReady(self):
        if self.reference_trj is None:
            return False
        if self.current_accel is None:
            return False
        if self.current_odometry is None:
            return False
        if self.crrent_velocity_report is None:
            return False
        if self.ego_pose is None:
                return False
        return True

    # Callback function for motion state subscriber
    def onMotion(self, msg: MotionState):
        self.motion_state = msg


    # Callback function for odometry subscriber
    def onOdometry(self, msg: Odometry):
        self.current_odometry = msg
        self.ego_pose = self.current_odometry.pose.pose
        self.crrent_vel = self.current_odometry.twist.twist.linear.x
        #self.get_logger().info("odometry {}".format(self.current_odometry.pose.pose))


    # Callback function for accrel subscriber
    def onAcceleration(self, msg: AccelWithCovarianceStamped):
        # return geometry_msgs/Accel 
        self.current_accel = accel = [msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z]


    # Callback function for velocity report subscriber
    def onVelocityReport(self, msg: VelocityReport):
        self.crrent_velocity_report = msg
        self.crrent_longitudinal_velocity = msg.longitudinal_velocity


    # Callback function for trajectory subscriber
    def onTrigger(self, msg: Trajectory):
        self.get_logger().info("Get trajectory. Processing crank driving planner...")
        self.reference_trj = msg
        
        if not self.isReady():
            print("Input data has not ready yet.")
            self.pub_trajectory_.publish(self.reference_trj)
            return
        
        
        self.get_logger().info("CurrentVel {}".format(self.crrent_vel))

        # If vehicle is not stopped, publish reference trajectory
        if self.crrent_longitudinal_velocity > 0 or self.crrent_vel > 0:
            points_accel_list = getAccelPointsFromTrajectory(self.reference_trj)
            points_vel_list = getVelocityPointsFromTrajectory(self.reference_trj)
            self.pub_trajectory_.publish(self.reference_trj)
        else:
            self.get_logger().info("Vehicle is stopped")
            
            points = self.reference_trj.points
            self.get_logger().info("Points num {}".format(len(points)))
            self.get_logger().info("Vehicle is stopp")
            points_vel_list = getVelocityPointsFromTrajectory(self.reference_trj)
            points_pose_list = getPosesFromTrajectory(self.reference_trj)
            points_accel_list = getAccelPointsFromTrajectory(self.reference_trj)
            
            ego_pose_ = ConvertPoint2List(self.ego_pose.position)

            n_p_idx = getNearestPointIndex(ego_pose_, points_pose_list)

            self.get_logger().info("Nearest point: {} {} {}".format(
                n_p_idx, points_pose_list[n_p_idx], points_vel_list[n_p_idx]))
            
            
            for k in range(15):
                self.reference_trj.points[n_p_idx + k].acceleration_mps2 = 0.0
                self.reference_trj.points[n_p_idx + k].longitudinal_velocity_mps = 5.0

            
            self.pub_trajectory_.publish(self.reference_trj)


def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()