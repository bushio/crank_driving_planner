import time
import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry

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

    # Callback function for odometry subscriber
    def onOdometry(self, msg: Odometry):
        self.current_odometry = msg
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
 

        # If vehicle is not stopped, publish reference trajectory
        if self.crrent_longitudinal_velocity > 0:
            self.pub_trajectory_.publish(self.reference_trj)
        else:
            self.get_logger().info("Vehicle is stopped")
            

            points = self.reference_trj.points
            self.get_logger().info("Points num {}".format(len(points)))
            self.get_logger().info("Vehicle is stopp")




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