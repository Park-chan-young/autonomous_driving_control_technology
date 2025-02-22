import numpy as np
import matplotlib.pyplot as plt
from lane_1 import lane

# Polynomial value calculation
def Polyval(coeff, x):
    num_coeff = len(coeff)
    y = coeff[0][0]
    for i in range(1, num_coeff):
        y += coeff[i][0]*(x**i)
    return y
        
# Global coordinate --> Local coordinate
def Global2Local(global_points, yaw_ego, X_ego, Y_ego):
    n = len(global_points)
    LocalPoints = np.zeros((n,2))
    
    for i in range(n):
        x_diff = global_points[i][0] - X_ego
        y_diff = global_points[i][1] - Y_ego
        LocalPoints[i][0] = x_diff * np.cos(yaw_ego) + y_diff * np.sin(yaw_ego)
        LocalPoints[i][1] = -x_diff * np.sin(yaw_ego) + y_diff * np.cos(yaw_ego)
    return LocalPoints
        
# Polynomial fitting (n_th order)
def Polyfit(points, num_order):
    m = num_order
    n = len(points)
    A = np.zeros((n, m+1))
    b = np.zeros((n, 1))
    coeff = np.zeros((m+1, 1))

    for i in range(n):
        x_val = points[i][0]
        y_val = points[i][1]
        for j in range(m+1):
            A[i][j] = x_val ** j
        b[i] = y_val

    coeff = np.linalg.lstsq(A, b, rcond=None)[0]
    return coeff

# Both lane to path
def BothLane2Path(coeff_L, coeff_R):
    num_coeff = len(coeff_L)
    coeff_path = np.zeros((num_coeff, 1))

    for i in range(num_coeff):
        average_coeff = (coeff_L[i][0] + coeff_R[i][0]) / 2
        coeff_path[i] = average_coeff
    return coeff_path

# Vehicle model
class VehicleModel_Lat(object):
    def __init__(self, step_time, Vx, m=500, L=4, kv=0.005, Pos=[0.0,0.0,0.0]):
        self.dt = step_time
        self.m = m
        self.L = L
        self.kv = kv
        self.vx = Vx
        self.yawrate = 0
        self.Yaw = Pos[2]
        self.X = Pos[0]
        self.Y = Pos[1]
    def update(self, delta, Vx):
        self.vx = Vx
        self.delta = np.clip(delta,-0.5,0.5)
        self.yawrate = self.vx/(self.L+self.kv*self.vx**2)*self.delta
        self.Yaw = self.Yaw + self.dt*self.yawrate
        self.X = self.X + Vx*self.dt*np.cos(self.Yaw)
        self.Y = self.Y + Vx*self.dt*np.sin(self.Yaw)

# Controller : Pure pursuit
class PurePursuit(object):
    def __init__(self, L=4.0, lookahead_time=1.0):
        self.L = L
        self.epsilon = 0.001
        self.t_lh = lookahead_time
    def ControllerInput(self, coeff, Vx):
        self.d_lh = Vx*self.t_lh
        self.y = Polyval(coeff, self.d_lh)
        self.u = np.arctan(2*self.L*self.y/(self.d_lh**2+self.y**2+self.epsilon))
        
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_lane = np.arange(0.0, 100.0, 0.1)
    Y_lane_L, Y_lane_R = lane(X_lane)
    
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
        Y_ref_L, Y_ref_R = lane(X_ref)
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
        coeff_path = BothLane2Path(coeff_L, coeff_R)
        # Controller input
        controller.ControllerInput(coeff_path, Vx)
        ego_vehicle.update(controller.u, Vx)
        
    plt.figure(1, figsize=(13,2))
    plt.plot(X_lane, Y_lane_L,'k--')
    plt.plot(X_lane, Y_lane_R,'k--',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Vehicle Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()
        
        