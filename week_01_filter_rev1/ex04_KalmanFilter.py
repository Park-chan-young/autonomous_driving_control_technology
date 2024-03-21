import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 칼만필터 클래스
class KalmanFilter:
    # model_variance: Q / measure_variance: R / error_variance_init: Px 초기값
    def __init__(self, y_measure_init, step_time=0.1, m=0.1, model_variance=0.01, measure_variance=1.0, error_variance_init=10.0):
        # A: state matrix
        self.A = np.array([1.0])
        # B: input matrix
        self.B = step_time / m
        # C: output matrix
        self.C = np.array([1.0])
        # D: direct transmission matrix
        self.D = np.array([0.0])
        # process noise 표준편차
        self.Q = model_variance
        # measurement noise 표준편차
        self.R = measure_variance
        # 초기 귀납적 추정치 = 측정치
        self.x_estimate = y_measure_init
        # Px: 초기 귀납적 오차 공분산
        self.P_estimate = error_variance_init

    def estimate(self, y_measure, input_u):
        # ****************** Prediction (귀납적 추정치 => 연역적 추정치 구하기) ******************
        # 연역적 추정치
        x_predict = self.A * self.x_estimate + self.B * input_u
        # 연역적 오차 공분산
        P_predict = self.A * self.P_estimate * self.A.T + self.Q

        # ****************** Update (연역적 추정치 + 측정치 => 귀납적 추정치 구하기) ******************
        # Kalman Gain
        K = P_predict * self.C.T / (self.C * P_predict * self.C.T + self.R)
        # 귀납적 추정치 (결국 이걸로 계산된 값이 최종 추정치로서 결과에 반영된다.)
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        # 귀납적 오차 공분산 / np.eye(n) => nxn 크기의 identity matrix / A.shape[0] => A의 행의 개수 / A.shape[1] => A의 열의 개수
        self.P_estimate = (np.eye(self.A.shape[0]) - K * self.C) * P_predict


if __name__ == "__main__":
    # Load data (엑셀 데이터를 불러오고)
    data = pd.read_csv("Data/example_KalmanFilter_1.csv")
    # Initialize KalmanFilter (필터를 초기화 한다.)
    kalman_filter = KalmanFilter(data.y_measure[0])

    # Estimate data
    # 계산 전 dataFrame의 'y_extimate'열의 데이터들을 결측치로 초기화 해주는 것 np.nan
    data['y_estimate'] = np.nan
    # 행, 열 수 만큼 반복 (y_estimate의 계산값을 update한다.)
    for i, row in data.iterrows():
        # KalmanFilter 클래스의 estimate 메소드를 사용하여 data의 y_measure, u열의 두개의 값으로 y_estimate를 계산하여 반환한다.
        kalman_filter.estimate(row['y_measure'], row['u'])
        # 반환 된 값을 y_estimate 열의 i번 째 행에 넣는다.
        data.at[i, 'y_estimate'] = kalman_filter.x_estimate

    print(data)

    # Plot results
    plt.figure('ex04_KalmanFilter')
    plt.plot(data.time, data.y_measure, 'k.', label="Measure")
    plt.plot(data.time, data.y_estimate, 'r-', label="Estimate")
    plt.xlabel('Time (s)')
    plt.ylabel('Signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()