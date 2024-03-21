from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class P_Controller(object):
    def __init__(self, P_Gain=0.5):
        self.kp = P_Gain
        
    def ControllerInput(self, reference, measure):
        self.measure = measure
        self.reference = reference
        # u = error * kp
        self.u = (self.reference-self.measure) * self.kp


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    time = []
    step_time = 0.1
    simulation_time = 30  
    # step time=0.1 / R=0 / force ratio=0.99 / force bias=0.1
    plant = VehicleModel(step_time, 0.0, 0.99, 0.1)
    controller = P_Controller()
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        controller.ControllerInput(target_y, plant.y_measure[0][0])
        plant.ControlInput(controller.u)
        print(plant.y_measure)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.title("P Controller")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
