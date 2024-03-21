import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 저주파통과필터 클래스
# 특별한 거 없이 평균필터, 이동평균 필터에서 사용한 내용들과 겹치는 부분이 대부분이다. 설명 안함.
class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):
        self.y_estimate = y_initial_measure
        self.alpha = alpha
 
    def estimate(self, y_measure):
        self.y_measure = y_measure
        self.y_estimate = self.alpha * self.y_estimate + (1 - self.alpha) * self.y_measure


if __name__ == "__main__":
    signal = pd.read_csv("Data/example_Filter_3.csv")

    y_estimate = LowPassFilter(signal.y_measure[0])

    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.y_estimate[i] = y_estimate.y_estimate

    print(signal)

    plt.figure('ex03_LowPassFilter')
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.title("alpha=0.9")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



