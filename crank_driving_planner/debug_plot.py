import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class PlotMarker():
    def __init__(self):
        self.plot_width = 50
        self.plot_forward = 20
        self.plot_backward = 40
        self.map_yaw = math.pi * 43/32

        self.map_rot = np.array([[math.cos(self.map_yaw), math.sin(self.map_yaw)],
                        [-math.sin(self.map_yaw), math.cos(self.map_yaw)]])     
        self.map_rot_wh = np.array([[math.cos(self.map_yaw), 0],
                                    [0, math.sin(self.map_yaw)]])
        
        self.ego_vec_length = 3
        self.robot_length = 3.5
        self.robot_width = 1.5

    def plot_status(self,
                    ego_pose = None,
                    object_pose = None,
                    left_bound = None,
                    right_bound = None,
                    path=None,
                    path_index_left=None,
                    path_index_next_left=None,
                    path_index_right= None,
                    path_index_next_right=None,
                    rotation = False,
                    predicted_goal_pose=None,
                    predicted_trajectory=None,
                    curve_plot=None,
                    curve_forward_point=None,
                    curve_backward_point=None,
                    vis_point=None
                    ):
        
        if ego_pose is not None:
            plt.cla()

            ego_pose_origin = ego_pose
            if rotation:
                ego_pose = np.array([0, 0])
            
            ## Plot ego pose
            plot_xmin = ego_pose[0] - self.plot_width / 2.0
            plot_xmax = ego_pose[0] + self.plot_width / 2.0
            plot_ymin = ego_pose[1] - self.plot_backward/ 2.0
            plot_ymax = ego_pose[1] + self.plot_forward / 2.0
            plt.plot(ego_pose[0], ego_pose[1], "xr")

            yaw = ego_pose_origin[2]
            ego_rot = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])

            outline = np.array([
                               [-self.robot_length / 2, self.robot_length / 2,
                                (self.robot_length / 2), -self.robot_length / 2,
                                -self.robot_length / 2],
                                [self.robot_width / 2, self.robot_width / 2,
                                - self.robot_width / 2, -self.robot_width / 2,
                                self.robot_width / 2]
                                ])
            outline = (outline.T.dot(ego_rot)).T
            
            outline[0, :] += ego_pose[0]
            outline[1, :] += ego_pose[1]
            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
            
            ## Plot allow
            ego_vec = np.array([np.cos(yaw), np.sin(yaw)]) * self.ego_vec_length
            ego_vec = ego_pose[0:2] + ego_vec
            trajec = np.array([[ego_pose[0], ego_pose[1]],
                              [ego_vec[0], ego_vec[1]]])
            plt.plot(trajec[:,0], trajec[:,1], "red")            

            ## Plot object pose
            if object_pose is not None:
                for ob in object_pose:
                    object_xy = ob[0:2] 
                    object_wh = ob[2:4]
                    if rotation:
                        object_xy = object_xy[0:2] - ego_pose_origin[0:2] 
                        object_xy = object_xy[0:2] @ self.map_rot
                        object_wh = object_wh[0:2] @ self.map_rot_wh

                    object_x_min = object_xy[0] - object_wh[0] / 2.0
                    object_y_min = object_xy[1] - object_wh[1] / 2.0
                    object_x_max = object_xy[0] + object_wh[0] / 2.0
                    object_y_max = object_xy[1] + object_wh[1] / 2.0
                    ob_box = np.array([[object_x_min, object_y_min],
                                       [object_x_min, object_y_max],
                                       [object_x_max, object_y_max],
                                       [object_x_max, object_y_min],
                                       [object_x_min, object_y_min]])

                    plt.plot(ob_box[:,0], ob_box [:,1], "blue")

            if path is not None:
                self.plot_path(path)


            ## Plot bound
            if (left_bound is not None) and (right_bound is not None):
                if rotation:
                    left_bound = left_bound[:,0:2] - ego_pose_origin[0:2] 
                    left_bound = left_bound[:,0:2] @ self.map_rot

                    right_bound = right_bound[:,0:2] - ego_pose_origin[0:2] 
                    right_bound = right_bound[:,0:2] @ self.map_rot

                plt.plot(left_bound[:, 0], left_bound[:, 1], color="black")
                plt.plot(right_bound[:,0], right_bound[:,1], color="black")

                if (path_index_left is not None) and (path_index_right is not None):
                    if path_index_next_left is None:
                        path_index_next_left = path_index_left + 1
                    if path_index_next_left is None:
                        path_index_next_left = path_index_right + 1

                    plt.plot(left_bound[path_index_left:path_index_next_left + 1 , 0], left_bound[path_index_left:path_index_next_left + 1, 1], color="green")
                    plt.plot(right_bound[path_index_right:path_index_next_right + 1, 0], right_bound[path_index_right:path_index_next_right + 1, 1], color="green")

            ## Plot goal pose
            if predicted_goal_pose is not None:
                self.plot_point(predicted_goal_pose)
            
            ## Plot trajectory
            if predicted_trajectory is not None:
                self.plot_traj(predicted_trajectory)

            ## Plot curve path
            if curve_plot is not None:
                self.plot_red_line(curve_plot)
            
            if curve_forward_point is not None:
                self.plot_point_delta(curve_forward_point)

            if curve_backward_point is not None:
                self.plot_point_delta(curve_backward_point)

            if vis_point is not None:
                self.vis_point(vis_point)


            plt.xlim(plot_xmin, plot_xmax)
            plt.ylim(plot_ymin, plot_ymax)
            plt.pause(0.01)
            #plt.show()

        else:
            return 

    def plot_red_line(self, line):
        plt.plot(line[:, 0], line[:, 1], color="Magenta")

    def plot_path(self, path):
        plt.plot(path[:, 0], path[:, 1], color="yellow", marker=".")
        #plt.plot(path[:, 0], path[:, 1], color="yellow")

    def plot_traj(self, traj):
        plt.plot(traj[:, 0], traj[:, 1], color="green", marker="^")

    def plot_point(self, point):
        plt.plot(point[0], point[1], marker="x")
    
    def plot_point_delta(self, point):
        plt.plot(point[0], point[1], marker="^",markersize=10)

    def vis_point(selfm, point):
        if len(point.shape) > 1:
            plt.scatter(point[:,0], point[:, 1], s=20, marker="D")
        else:
            plt.plot(point[0], point[1], marker="8",markersize=8)
        
