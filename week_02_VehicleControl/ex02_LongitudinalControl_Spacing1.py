import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

class PID_Controller_ConstantSpace(object):
    def __init__(self, step_time, target_x, ego_x, constantSpace=30.0, P_Gain=0.4, D_Gain=4.0, I_Gain=0.0):
        self.step_time = step_time
        self.target_x = target_x
        self.ego_x = ego_x
        self.constantSpace = constantSpace
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.prev_error = 0.0
    
    def ControllerInput(self, target_x, ego_x):
        self.target_x = target_x
        self.ego_x = ego_x
        self.error = (self.target_x - self.ego_x) - self.constantSpace
        P_term = self.error * self.P_Gain
        D_term = (self.error - self.prev_error) * self.D_Gain / self.step_time
        I_term = self.I_Gain * self.error * self.step_time
        self.u = P_term + D_term + I_term
        self.prev_error = self.error

        

if __name__ == "__main__":
    
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0
    
    vx_ego = []
    vx_target = []
    x_space = []
    time = []
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    ego_vehicle = VehicleModel_Long(step_time, m, 0.5, 0.0, 10.0)
    controller = PID_Controller_ConstantSpace(step_time, target_vehicle.x, ego_vehicle.x)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        vx_ego.append(ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(target_vehicle.x - ego_vehicle.x)
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x)
        ego_vehicle.update(controller.u)
        target_vehicle.update(0.0)

        print('------------')
        print(controller.u)
        
        
    plt.figure('velocity')
    plt.plot(time, vx_ego,'r-',label = "ego_vx [m/s]")
    plt.plot(time, vx_target,'b-',label = "target_vx [m/s]")
    plt.xlabel('time [s]')
    plt.ylabel('Vx')
    plt.legend(loc="best")
    plt.title("Constant Spacing Control (Velocity)")
    plt.axis("equal")
    plt.grid(True)
    
    plt.figure('position')
    plt.plot([0, time[-1]], [controller.constantSpace, controller.constantSpace], 'k-', label="reference")
    plt.plot(time, x_space,'b-',label = "space [m]")
    plt.xlabel('time [s]')
    plt.ylabel('x')
    plt.legend(loc="best")
    plt.title("Constant Spacing Control (Position)")
    plt.axis("equal")
    plt.grid(True)    
    
    plt.show()

    