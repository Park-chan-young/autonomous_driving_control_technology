from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PD_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.6, D_Gain=100.2):
        self.reference = reference
        self.measure = measure
        self.step_time = step_time
        self.kp = P_Gain
        self.kd = D_Gain
        # prev_measure의 초기값은 measure
        self.prev_error = self.reference - self.measure
    
    def ControllerInput(self, reference, measure):
        self.reference = reference
        self.measure = measure
        self.error = self.reference - self.measure
        # u = error * kp + d(error)/dt * kd         # d(error) = 현재error - 과거error
        self.u = self.error * self.kp + self.kd * (self.error - self.prev_error) / self.step_time
        self.prev_error = self.error


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1)
    controller = PD_Controller(target_y, plant.y_measure[0][0], step_time, P_Gain=0.6, D_Gain=1.2)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u)
        print(plant.y_measure, controller.u)

    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.title("PD Controller")
    plt.axis("equal")
    plt.grid(True)
    plt.show()