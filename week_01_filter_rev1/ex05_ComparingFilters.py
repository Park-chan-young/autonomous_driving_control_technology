# 앞에서 작성한 각 Filter class 들을 가져와서 사용한다.
from ex01_AverageFilter import AverageFilter
from ex02_MovingAverageFilter import MovingAverageFilter
from ex03_LowPassFilter import LowPassFilter
from ex04_KalmanFilter import KalmanFilter
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import queue


if __name__ == "__main__":
    # 각 필터들에 대한 결과를 리스트로 저장한다.
    t = []
    y_AF = []
    y_MAF = []
    y_LPF = []
    y_KF = []
    
    # signal 이라는 인스턴스에 시험 데이터를 저장한다.
    signal = pd.read_csv("Data/example_KalmanFilter_1.csv")

    # instantiate filters
    # 각각의 class를 사용하여 필터 가동하여 y_extimate 저장할 인스턴스 생성
    y_estimate_AF = AverageFilter(signal.y_measure[0])
    y_estimate_MAF = MovingAverageFilter(signal.y_measure[0])
    y_estimate_LPF = LowPassFilter(signal.y_measure[0])
    y_estimate_KF = KalmanFilter(signal.y_measure[0])

    
    for i, row in signal.iterrows():
        # append(): 리스트 맨 뒤에 새로운 요소를 추가하는 함수
        t.append(signal.time[i])
        # Averageg filter의 추정치 리스트 채우기
        y_estimate_AF.estimate(signal.y_measure[i])
        y_AF.append(y_estimate_AF.y_estimate)
        # Moving Average filter의 추정치 리스트 채우기
        y_estimate_MAF.estimate(signal.y_measure[i])
        y_MAF.append(y_estimate_MAF.y_estimate)
        # Low pass filter의 추정치 리스트 채우기
        y_estimate_LPF.estimate(signal.y_measure[i])
        y_LPF.append(y_estimate_LPF.y_estimate)
        # Kalman filter의 추정치 리스트 채우기
        y_estimate_KF.estimate(signal.y_measure[i],signal.u[i])
        # KalmanFilter의 계산결과가 2차원 array로 나와 plot이 불가 => item()함수 사용하여 다차원 요소를 scalar 값으로 변환
        y_KF.append(y_estimate_KF.x_estimate.item())

    # print(y_AF)
    # print(y_MAF)
    # print(y_LPF)
    # print(y_KF)  

    plt.figure("ex05 Comparing Filter")
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(t, y_AF,'m-',label = "Average Filter")
    plt.plot(t, y_MAF,'b-',label = "Moving Average Filter")
    plt.plot(t, y_LPF,'c-',label = "Low Pass Filter")
    plt.plot(t, y_KF,'r-',label = "Kalman Filter")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
