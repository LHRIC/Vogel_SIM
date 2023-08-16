import numpy as np


class PTN:
    def __init__(self):
        # 6200:100:14100
        self.torque_mod = 1
        self.rpm_range = np.array([*range(6200, 14100 + 100, 100)])
        self.torque_curve = self.torque_mod * np.array([41.57, 42.98, 44.43, 45.65, 46.44, 47.09, 47.52, 48.58, 49.57, 50.41, 51.43, 51.48, 51, 49.311, 48.94, 48.66, 49.62, 49.60, 47.89, 47.91, 48.09, 48.57, 49.07, 49.31, 49.58, 49.56, 49.84, 50.10, 50.00, 50.00, 50.75, 51.25, 52.01, 52.44, 52.59, 52.73, 53.34, 53.72,
                             52.11, 52.25, 51.66, 50.5, 50.34, 50.50, 50.50, 50.55, 50.63, 50.17, 50.80, 49.73, 49.35, 49.11, 48.65, 48.28, 48.28, 47.99, 47.68, 47.43, 47.07, 46.67, 45.49, 45.37, 44.67, 43.8, 43.0, 42.3, 42.00, 41.96, 41.70, 40.43, 39.83, 38.60, 38.46, 37.56, 36.34, 35.35, 33.75, 33.54, 32.63, 31.63])
        self.primary_reduction = 76/36
        self.gear_ratios = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24]
        self.num_gears = len(self.gear_ratios)
        self.final_drive = 37/11
        self.shiftpoint = 12500
        self.drivetrain_losses = 0.85
        self.shift_time = 0.25 #seconds
        self.diff_locked = False; #False - open, True - closed


if __name__ == "__main__":
    a = PTN()
    print(len(a.rpm_range))
    print(len(a.torque_curve))