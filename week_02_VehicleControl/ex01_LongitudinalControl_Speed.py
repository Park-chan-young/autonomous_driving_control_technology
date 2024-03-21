import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long

#  정상상태 오차를 0으로 하기 위해 PI 제어기를 사용한다.
class PID_Controller_Speed(object):
    def __init__(self, reference, measure, step_time, P_Gain=4.0, D_Gain=0.0, I_Gain=0.0):
        self.reference = reference
        self.measure = measure
        self.step_time = step_time
        self.P_Gain = P_Gain
        self.D_Gain = D_Gain
        self.I_Gain = I_Gain
        self.prev_error = self.reference - self.measure

    def ControllerInput(self, reference, measure):
        self.reference = reference
        self.measure = measure
        self.error = self.reference - self.measure
        P_term = self.error * self.P_Gain
        D_term = (self.error - self.prev_error) * self.D_Gain / self.step_time
        I_term = self.I_Gain * self.error * self.step_time
        self.u = P_term + D_term + I_term
        self.prev_error = self.error


if __name__ == "__main__":
    
    step_time = 0.1
    simulation_time = 50.0
    m = 500.0
    

    reference_speed = 30.0
    Vx = []
    ax = []
    time = []
    plant = VehicleModel_Long(step_time, m, 0.5, 0.0, 0.0)
    controller = PID_Controller_Speed(reference_speed, plant.vx, step_time)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        Vx.append(plant.vx)
        ax.append(plant.ax)
        controller.ControllerInput(reference_speed,plant.vx)
        plant.update(controller.u)
    
        print("----")
        print(plant.ax)
        print(controller.u)
        
    plt.figure(1)
    plt.plot(time, Vx,'r-',label = "Vx")
    plt.xlabel('time [s]')
    plt.ylabel('Vx [m/s]')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.title("Speed Control (P Controller; kp=4)")
    plt.grid(True)
    plt.show()
