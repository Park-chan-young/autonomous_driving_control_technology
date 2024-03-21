from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.5, D_Gain=4.0, I_Gain=0.0):
        self.reference = reference
        self.measure = measure
        self.step_time = step_time
        self.kp = P_Gain
        self.kd = D_Gain
        self.ki = I_Gain
        self.prev_error = self.reference - self.measure
        self.i_term = 0
        self.u = 0
    
    def ControllerInput(self, reference, measure):
        self.reference = reference
        self.measure = measure
        error = self.reference - self.measure
        p_term = self.kp * error
        d_term = self.kd * (error - self.prev_error) / self.step_time
        self.i_term += self.ki * error * self.step_time
        self.u = p_term + d_term + self.i_term
        self.prev_error = error
        
class KalmanFilter:
   def __init__(self, y_Measure_init, step_time=0.1, m=10.0, Q_x=0.02, Q_v=0.05, R=4.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0],[step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.P_estimate = np.array([[errorCovariance_init, 0.0],[0.0, errorCovariance_init]])
       
   def estimate(self, y_measure, input_u):
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A.T + self.Q

        K = P_predict * self.C.T / (self.C * P_predict * self.C.T + self.R)
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (np.eye(self.A.shape[0]) - K * self.C) * P_predict
        
        
if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.x_estimate[0][0])
        estimator.estimate(plant.y_measure[0][0],controller.u)
        controller.ControllerInput(target_y, estimator.x_estimate[0][0])
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r:',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
