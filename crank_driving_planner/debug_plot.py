import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class PlotMarker():
    def __init__(self):
        self.plot_width = 40
        self.plot_height = 40

    def plot_status(self,
                    ego_pose = None,
                    object_pose = None,
                    left_bound = None,
                    right_bound = None,
                    rotation = True,
                    axis_yx = False
                    ):
        yaw = math.pi * 43/32
        if ego_pose is not None:
            plt.cla()

            if rotation:
                rot = np.array([[math.cos(yaw), math.sin(yaw)],
                                [-math.sin(yaw), math.cos(yaw)]])
                ego_pose_origin = ego_pose[0:2] 
                ego_pose = np.array([0, 0])
            
            if axis_yx:
                plot_xmin = ego_pose[1] - self.plot_height/ 2.0
                plot_xmax = ego_pose[1] + self.plot_height / 2.0
                plot_ymin = ego_pose[0] - self.plot_width / 2.0
                plot_ymax = ego_pose[0] + self.plot_width / 2.0
                plt.plot(ego_pose[1], ego_pose[0], "xr")
            else:
                plot_xmin = ego_pose[0] - self.plot_width / 2.0
                plot_xmax = ego_pose[0] + self.plot_width / 2.0
                plot_ymin = ego_pose[1] - self.plot_height / 2.0
                plot_ymax = ego_pose[1] + self.plot_height / 2.0
                plt.plot(ego_pose[0], ego_pose[1], "xr")

            if object_pose is not None:
                if rotation:
                    object_pose = object_pose[:,0:2] - ego_pose_origin
                    object_pose = object_pose[:,0:2] @ rot

                if axis_yx:
                    plt.plot(object_pose[:,1], object_pose[:,0], "ob")
                else:
                    plt.plot(object_pose[:,0], object_pose[:,1], "ob")

            if left_bound is not None:
                if rotation:
                    left_bound = left_bound[:,0:2] - ego_pose_origin
                    left_bound = left_bound[:,0:2] @ rot

                if axis_yx:
                    plt.plot(left_bound[:,1], left_bound[:,0], color="black")
                else:
                    plt.plot(left_bound[:,0], left_bound[:,1], color="black")

            if right_bound is not None:
                if rotation:
                    right_bound = right_bound[:,0:2] - ego_pose_origin
                    right_bound = right_bound[:,0:2] @ rot

                if axis_yx:
                    plt.plot(right_bound[:,1], right_bound[:,0], color="black")
                else:
                    plt.plot(right_bound[:,0], right_bound[:,1], color="black")

            plt.xlim(plot_xmin, plot_xmax)
            plt.ylim(plot_ymin, plot_ymax)
            #plt.grid(True)
            plt.pause(0.01)
            #plt.show()

        else:
            return 



