import numpy as np
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import PredictedObjectKinematics
from autoware_auto_perception_msgs.msg import Shape

import matplotlib.pyplot as plt


class PredictedObjectsInfo:
    def __init__(self, predicted_obj: PredictedObjects, show_objects_plot=False):
        self.objects_poses = np.empty((0,2)) ## Store each points (x, y)
        self.show_objects_plot = show_objects_plot
        self.objects_rectangle = np.empty((0,4))
        self.car_shape = [[1.0, 0.5],[-1.0, -0.5]]
        self.objects_vis_box =  []

        self.objects_labels = []
        for idx in range(len(predicted_obj)):
            ob = predicted_obj[idx]
            ## Get 2-D pose
            pose = self._getInitialPosesfromPredictedObjectKinematics(ob.kinematics)
            self.objects_poses = np.vstack([self.objects_poses, pose])

            ## Get object shape
            shape_array = self._getShapeArrayfromPredictedObjectKinematics(ob.shape)
            
            if len(shape_array) == 0:
                shape_array = self.car_shape

            object_points = shape_array  + pose

            xmin = np.min(object_points[:, 0])
            ymin = np.min(object_points[:, 1])
            xmax = np.max(object_points[:, 0])
            ymax = np.max(object_points[:, 1])
            cx = (xmax + xmin)/ 2.0
            cy = (ymax + ymin)/ 2.0
            length_x = abs(xmax - xmin)
            length_y = abs(ymax - ymin)
            rect = np.array([cx, cy ,length_x ,length_y])
            vis_box = [[xmin, ymin], [xmin, ymax], [xmax, ymax], [xmax, ymin], [xmin, ymin]]

            self.objects_vis_box.append(vis_box)
            self.objects_rectangle = np.vstack([self.objects_rectangle, rect])

            if self.show_objects_plot:
                object_points = np.vstack([object_points, object_points[0]])
                self.show_objects(self.objects_rectangle)

        if self.objects_vis_box is not None:
            self.objects_vis_box = np.array(self.objects_vis_box)

    def _getInitialPosesfromPredictedObjectKinematics(self, kinematics: PredictedObjectKinematics):
        x = kinematics.initial_pose_with_covariance.pose.position.x
        y = kinematics.initial_pose_with_covariance.pose.position.y
        return np.array([x, y])
    

    def _getShapeArrayfromPredictedObjectKinematics(self, ob_shape: Shape):
        shape_array = np.empty((0,2))
        for idx in range(len(ob_shape.footprint.points)):
            p = ob_shape.footprint.points[idx]
            shape_array = np.vstack([shape_array, [p.x, p.y]])
        return shape_array
    

    def show_objects(self, object_points):
        plt.plot(object_points[:,0],object_points[:,1])
        plt.show()