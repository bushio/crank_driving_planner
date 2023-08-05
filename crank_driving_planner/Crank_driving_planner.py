import time
import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Trajectory

class CrankDrigingPlanner(Node):
    def __init__(self):
        super().__init__('CrankDrigingPlanner')
        self.get_logger().info("Start CrankDrigingPlanner")
        
        # Reference trajectory subscriber. Remap "obstacle_stop_planner/trajectory"
        self.create_subscription(Trajectory ,"~/input/trajectory", self.onTrigger, 1)


        # Trajectory publisher. Remap "/planning/scenario_planning/lane_driving/trajectory"
        self.pub_trajectory_ = self.create_publisher(Trajectory, 
                                                     "~/output/trajectory", 
                                                     1 )
        
        self.reference_trj = None
    
    def onTrigger(self, msg: Trajectory):
        self.get_logger().info("Get trajectory. Processing crank driving planner...")
        self.reference_trj = msg

        self.pub_trajectory_.publish(self.reference_trj)
        #print(dir(self.reference_trj))

def main(args=None):
    print('Hi from CrankDrigingPlanner')
    rclpy.init(args=args)
    
    crank_drining_plan_class = CrankDrigingPlanner()
    rclpy.spin(crank_drining_plan_class)

    crank_drining_plan_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()