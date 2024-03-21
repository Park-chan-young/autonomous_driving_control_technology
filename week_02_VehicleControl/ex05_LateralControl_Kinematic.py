import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat

class PID_Controller_Kinematic(object):
    def __init__(self, step_time, Y_ref, Y, P_Gain=5.0, D_Gain=0.5, I_Gain=0.0):
        self.step_time = step_time
        self.Y_ref = Y_ref
        self.Y = Y
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.prev_error = 0.0

    def ControllerInput(self, Y_ref, Y):
        self.Y_ref = Y_ref
        self.Y = Y
        self.error = self.Y_ref - self.Y
        P_term = self.error * self.P_Gain
        D_term = (self.error - self.prev_error) * self.D_Gain / self.step_time
        I_term = self.I_Gain * self.error * self.step_time
        self.u = P_term + D_term + I_term
        self.prev_error = self.error

    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    Y_ref = 4.0
    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)   
    controller = PID_Controller_Kinematic(step_time, Y_ref, ego_vehicle.Y)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        controller.ControllerInput(Y_ref, ego_vehicle.Y)
        ego_vehicle.update(controller.u, Vx)  # def update(self, delta, Vx): 여기서 u는 delta이다!!
        print(ego_vehicle.X, ego_vehicle.Y)

        
    plt.figure(1)
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.plot([0, X_ego[-1]], [Y_ref, Y_ref], 'k:',label = "Reference")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.title("Lateral (Kinematic model)")
    # plt.axis("best")
    plt.grid(True)    
    plt.show()


