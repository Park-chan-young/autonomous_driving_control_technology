import numpy as np
import matplotlib.pyplot as plt
from lane_2 import lane
from ex01_PathPlanning_BothLane import Global2Local, Polyfit, VehicleModel_Lat, PurePursuit

class LaneWidthEstimator(object):
    def __init__(self, Lw_init=3.0):
        self.Lw = Lw_init

    def update(self, coeff_L, coeff_R, isLaneValid_L, isLaneValid_R):
        if all(isLaneValid_L) and all(isLaneValid_R):
            self.Lw = coeff_L[0][0] - coeff_R[0][0]
        else:
            self.Lw = 4.0
        return self.Lw


def EitherLane2Path(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R, Lw):
    num_coeff = len(coeff_L)
    coeff_path = np.zeros((num_coeff, 1))

    if all(isLaneValid_L) and all(isLaneValid_R):
        for i in range(num_coeff):
            average_coeff = (coeff_L[i][0] + coeff_R[i][0]) / 2
            coeff_path[i] = average_coeff

    elif all(isLaneValid_L) and not any(isLaneValid_R):
        coeff_R = coeff_L
        coeff_R[0][0] = coeff_L[0][0] - Lw/2
        for i in range(num_coeff):
            average_coeff = (coeff_L[i][0] + coeff_R[i][0]) / 2
            coeff_path[i] = average_coeff

    elif all(isLaneValid_R) and not any(isLaneValid_L):
        coeff_L = coeff_R
        coeff_L[0][0] = coeff_R[0][0] + Lw/2
        for i in range(num_coeff):
            average_coeff = (coeff_L[i][0] + coeff_R[i][0]) / 2
            coeff_path[i] = average_coeff

    return coeff_path

if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_lane = np.arange(0.0, 100.0, 0.1)
    Y_lane_L, Y_lane_R, Valid_L, Valid_R = lane(X_lane)
    
    LaneWidth = LaneWidthEstimator()
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PurePursuit()
    
    time = []
    X_ego = []
    Y_ego = []
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        # Lane Info
        X_ref = np.arange(ego_vehicle.X, ego_vehicle.X+5.0, 1.0)
        Y_ref_L, Y_ref_R, isLaneValid_L, isLaneValid_R = lane(X_ref)
        # Global points (front 5 meters from the ego vehicle)
        global_points_L = np.transpose(np.array([X_ref, Y_ref_L])).tolist()
        global_points_R = np.transpose(np.array([X_ref, Y_ref_R])).tolist()
        # Converted to local frame
        local_points_L = Global2Local(global_points_L, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        local_points_R = Global2Local(global_points_R, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        # 3th order fitting
        coeff_L = Polyfit(local_points_L, num_order=3)
        coeff_R = Polyfit(local_points_R, num_order=3)
        # Lane to path
        LaneWidth.update(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R)
        coeff_path = EitherLane2Path(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R, LaneWidth.Lw)
        # Controller input
        controller.ControllerInput(coeff_path, Vx)
        ego_vehicle.update(controller.u, Vx)
        print("iteration => ", i)
        print("Lw => ", LaneWidth.Lw)
        print("isValid_L => ", isLaneValid_L)
        
    plt.figure(1, figsize=(13,2))
    plt.plot(X_lane, Y_lane_L,'k--')
    plt.plot(X_lane, Y_lane_R,'k--',label = "Reference")
    plt.plot(X_ego, Y_ego,'b.',label = "Vehicle Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()