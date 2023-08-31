

class CurveConfig:
    def __init__(self):
        self.circle_radius = [4.0, 4.5, 3.0, 4.5] #[m]
        self.circle_angle_rate = [0.7, 0.6, 0.4, 0.3] 

        self.inner_start_mergin = [2.0, 2.0, 1.5, 0.1] #[m]
        self.inner_finish_mergin = [5.0, 5.0, 1.0, 1.0] #[m]

        self.exec_predict = [False, False, False, True]

        self.sharp_dist_threshold = 17 #[m]
        self.animation_flag = False

        self.arrival_threthold = 1.5 #[m]