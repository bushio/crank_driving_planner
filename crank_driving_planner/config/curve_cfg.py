

class CurveConfig:
    def __init__(self):
        self.circle_radius = [4.0, 4.5, 3.0, 7.5]
        self.circle_angle_rate = [0.7, 0.6, 0.5, 0.3]
        self.inner_start_mergin = [2.0, 2.0, 2.0, 3.5]
        self.exec_predict = [False, True, True, True]

        self.sharp_dist_threshold = 15