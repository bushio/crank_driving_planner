import math

class DWA_Config:
    """
    simulation parameter class
    """

    def __init__(self):
        ################
        # Simulator parameter
        #################        
        # Matplotlib parameter
        self.plot_y_min = -2
        self.plot_y_max = 40
        self.plot_x_min = 0
        self.plot_x_max = 40
        
        # Path parameter
        self.point_interval = 0.2 #[m]

        # Robot parameter
        self.robot_type = "rectangle" #  "rectangle" or "circle"
        self.robot_radius = 1.0 # [m]
        self.robot_width = 1.5  # [m]
        self.robot_length = 3.5  # [m]
        
        ################
        # DWA parameter
        #################
        # robot parameter
        self.max_speed = 3.0  # [m/s]
        self.min_speed = -1.0  # [m/s]
        self.max_yaw_rate = 35.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1.0 # [m/ss]
        self.max_delta_yaw_rate = 35.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.25  # [s] Time tick for motion prediction
        
        # Resolution of rotation motion
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        # Resolution of translational motion
        self.v_resolution = 0.1  # [m/s]
        
        # Prediction time. The longer this time, the longer trajectory length.
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 10.0
        self.path_cost_gain = 10.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        
        # If the distance between object and robot more than threshold , cost is ignored.
        self.ob_dist_threshold = 10.0
        self.path_dist_threshold = 10.0

        self.ob_penalty = -1 # If vehicle stacked obstacle, the cost is set as this value.
        self.path_penalty = -1 # If vehicle stacked path, the cost is set as this value.
        self.path_point_size = 0.1 # vehicle and path point margin 