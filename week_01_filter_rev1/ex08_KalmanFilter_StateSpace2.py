import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=10.0, k=100.0, b=2.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [-k*step_time/m, 1 - b*step_time/m]])
        self.B = np.array([[0.0], [step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init],[0.0]])
        self.P_estimate = np.array([[errorCovariance_init, 0.0],[0.0, errorCovariance_init]])

    def estimate(self, y_measure, input_u):
        # Prediction
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A.T + self.Q

        # Update
        K = P_predict * self.C.T / (self.C * P_predict * self.C.T + self.R)
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (np.eye(self.A.shape[0]) - K * self.C) * P_predict
        
if __name__ == "__main__":
    signal = pd.read_csv("Data/example08.csv")
    y_estimate = KalmanFilter(y_Measure_init=signal.y_measure[0])

    signal['y_estimate'] = np.nan  
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate[0][0]

    print(signal)

    plt.figure('ex8_KalmanFilter_StateSpace2')
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



