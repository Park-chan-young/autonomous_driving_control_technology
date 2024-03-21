import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_measure_init, step_time=0.1, m=0.1, model_variance=0.0001, measure_variance=100.0, error_variance_init=10.0):
        self.A = np.array([1.0])
        self.B = step_time / m
        self.C = np.array([1.0])
        self.D = np.array([0.0])
        self.Q = model_variance
        self.R = measure_variance
        self.x_estimate = y_measure_init
        self.P_estimate = error_variance_init

    def estimate(self, y_measure, input_u):
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A.T + self.Q

        K = P_predict * self.C.T / (self.C * P_predict * self.C.T + self.R)
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (np.eye(self.A.shape[0]) - K * self.C) * P_predict


if __name__ == "__main__":
    signal = pd.read_csv("Data/example06.csv")
    y_estimate = KalmanFilter(y_measure_init=signal.y_measure[0])

    signal['y_estimate'] = np.nan
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate

    print(signal)

    plt.figure('ex6_TuningKalmanFilter')
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
