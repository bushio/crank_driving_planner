import math
import numpy as np

class DynamicWindowApproach:
    def __init__(self, cfg):
        ## Robot parameter
        self.min_speed = cfg.min_speed
        self.max_speed = cfg.max_speed
        self.max_yaw_rate = cfg.max_yaw_rate
        self.max_yaw_rate = cfg.max_yaw_rate
        self.max_accel = cfg.max_accel
        self.max_delta_yaw_rate = cfg.max_delta_yaw_rate
        
        self.dt = cfg.dt
        
        ## Search range
        self.v_resolution = cfg.v_resolution
        self.yaw_rate_resolution = cfg.yaw_rate_resolution
        
        ## Cost weights
        self.speed_cost_gain = cfg.speed_cost_gain
        self.to_goal_cost_gain = cfg.to_goal_cost_gain
        self.obstacle_cost_gain = cfg.obstacle_cost_gain
        self.path_cost_gain = cfg.path_cost_gain
        
        # If the distance between object and robot more than threshold , cost is ignored.
        self.ob_dist_threshold = cfg.ob_dist_threshold
        self.path_dist_threshold = cfg.path_dist_threshold
        self.ob_penalty = cfg.ob_penalty
        self.path_penalty = cfg.path_penalty
        
        ## Trajectory precition time 
        self.predict_time = cfg.predict_time
    
        ## Robot param
        self.robot_radius = cfg.robot_radius # [m]
        self.robot_width = cfg.robot_width  # [m]
        self.robot_length = cfg.robot_length  # [m]
        self.robot_stuck_flag_cons = cfg.robot_stuck_flag_cons
        
    def get_next_step(self, x, goal, obstacle, left_bound, right_bound):
        """
        Return next control and trajectory
        """
        
        # Calcuration dynamic window by self property
        motion_dw = self._calc_dynamic_window(x)
        
        u, trajectory = self.calc_control_and_trajectory(x, motion_dw, goal,  obstacle, left_bound, right_bound)
        return u, trajectory
    
    
    def _calc_dynamic_window(self, x):
        """
        Calculation dynamic window based on current state x
        """

        ## Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        ## Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw
    

    def calc_control_and_trajectory(self, x, dw, goal, ob, left_bound, right_bound):
        """
        Calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        ## Evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                
                ## Get next trajectory
                trajectory = self.predict_trajectory(x_init, v, y)
                
                ## Calculate cost
                to_goal_cost = self.to_goal_cost_gain * self._calc_target_heading_cost(trajectory[-1], goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])

                ob_cost, tp_idx_obj = self._calc_obstacle_cost(
                    trajectory, ob, dist_threshold=self.ob_dist_threshold, penalty=self.ob_penalty)
                
                path_cost, tp_idx_path = self._calc_path_cost(
                        trajectory, left_bound, right_bound, dist_threshold=self.path_dist_threshold, penalty=self.path_penalty)

                tp_idx = min(tp_idx_obj, tp_idx_path)
                trajectory = trajectory[:tp_idx]

                final_cost = to_goal_cost + speed_cost + self.obstacle_cost_gain * ob_cost + self.path_cost_gain * path_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        print(min_cost)
        return best_u, best_trajectory
    
    
    def predict_trajectory(self, x_init, v, y):
        """
        Predict next trajectory
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self._motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory


    def _motion(self, x, u, dt):
        """
        Calculate motion model
        """
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x


    def _calc_target_heading_cost(self, final_pose, target):
        """
            heading is a measure of progress towards the
            goal location. It is maximal if the robot moves directly towards the target.
        """
        dx = target[0] - final_pose[0]
        dy = target[1] - final_pose[1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - final_pose[2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost


    def _calc_obstacle_cost(self, trajectory, objects, dist_threshold=5.0, penalty=-1):        
        object_xy = objects[:, 0:2]
        object_r = 2.0
        
        min_dist = float("Inf")
        if penalty == -1:
            penalty = float("Inf")
       
        ## Distance between object and trajectory points
        for idx, tp in enumerate(trajectory):
            ox = objects[:, 0]
            oy = objects[:, 1]
            dx = tp[0] - ox
            dy = tp[1] - oy
            dist = np.hypot(dx, dy)
            
            # Mask with distance
            mask = dist < dist_threshold
            object_xy_mask = object_xy[mask, :]
            
            if not object_xy_mask.any():
                dist = 1000

            if min_dist < np.min(dist):
                min_dist = dist
                
            collision_mask = object_r > dist
            object_xy_collision = object_xy[collision_mask, :]
            check = object_xy_collision.any()
            if check:
                return penalty, idx
            
        return 1.0 / min_dist, len(trajectory) # OK
        

    def _calc_path_cost(self, trajectory, left_bound, right_bound, path_point_size=0.1, dist_threshold=5.0, penalty=-1):

        left_bound_vec = np.diff(left_bound, axis=0)
        right_bound_vec = np.diff(right_bound, axis=0)

        if penalty == -1:
            penalty = float("Inf")
        min_dist = float("Inf")

        ## Distance between object and trajectory points
        ## If teh outer product is >  0, the point is to the left of the bound.
        ## and if teh outer product is <  0, the point is to the right of the bound.
        for idx, tp in enumerate(trajectory):
            tp_point = np.array([tp[0], tp[1]])
            tp_vec_left = tp_point - left_bound[:, 0:2]
            tp_vec_right = tp_point - right_bound[:, 0:2]
            left_check = np.cross(left_bound_vec, tp_vec_left[:len(left_bound_vec), 0:2])
            right_check = np.cross(right_bound_vec, tp_vec_right[:len(right_bound_vec), 0:2])
            

            ## Calc distance between trajectory_pose and lef_bound using traiangle area 
            for ii, v in enumerate(left_bound_vec):
                traiangle_bottom = np.hypot(v[0], v[1])
                p1 = left_bound[ii]
                p2 = left_bound[ii + 1]
                triangle_area = self._calcTriangleArea(tp[0], tp[1], p1[0], p1[1], p2[0], p2[1])
                dist = (triangle_area * 2) / traiangle_bottom
                if min_dist > dist:
                    min_dist = dist
                    
            ## Calc distance between trajectory_pose and right_bound using traiangle area 
            for ii, v in enumerate(right_bound_vec):
                traiangle_bottom = np.hypot(v[0], v[1])
                p1 = right_bound[ii]
                p2 = right_bound[ii + 1]
                triangle_area = self._calcTriangleArea(tp[0], tp[1], p1[0], p1[1], p2[0], p2[1])
                dist = (triangle_area * 2) / traiangle_bottom
                
                if min_dist > dist:
                    min_dist = dist

             ## If any points is beyond left_bound.
            if np.any(left_check > 0):
                return min_dist, idx
            
            if np.any(right_check < 0):
                return min_dist, idx
            
        return 1.0 / min_dist, len(trajectory)  # OK
    

    def _calcTriangleArea(self, x1, y1, x2, y2, x3, y3):
        return 0.5 * abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
