from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.4, D_Gain=0.9, I_Gain=0.02):
        self.reference = reference
        self.measure = measure
        self.step_time = step_time
        self.kp = P_Gain
        self.kd = D_Gain
        self.ki = I_Gain
        self.prev_error = self.reference - self.measure
        self.i_term = 0
    
    def ControllerInput(self, reference, measure):
        self.reference = reference
        self.measure = measure
        error = self.reference - self.measure
        p_term = self.kp * error
        d_term = self.kd * (error - self.prev_error) / self.step_time
        self.i_term += self.ki * error * self.step_time
        self.u = p_term + d_term + self.i_term
        self.prev_error = error
        
class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.alpha = alpha
 
    def estimate(self, y_measure):
        self.y_measure = y_measure
        self.y_estimate = self.alpha * self.y_estimate + (1 - self.alpha) * self.y_measure


if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = LowPassFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.y_estimate)
        estimator.estimate(plant.y_measure[0][0])
        # controller에서 measure이 아니라 estimate로 계산된 결과를 반환한다.
        controller.ControllerInput(target_y, estimator.y_estimate)
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r-',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
