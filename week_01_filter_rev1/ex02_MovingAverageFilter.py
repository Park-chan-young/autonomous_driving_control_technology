import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# Buffer역할을 할 큐(queue)를 만들기 위해 가져온 라이브러리이다.
import queue

# 이동평균필터 클래스
class MovingAverageFilter:
    # 클래스를 초기화 해주는데, 평균계산할 데이터의 개수 n을 여기서 바로 지정한다.
    def __init__(self, y_initial_measure, n=30):
        # y_Buffer라는 이름의 queue를 만들었다. (FIFO 방식으로 데이터를 저장한다!!!)
        # FIFO 방식의 데이터 처리를 도와주는 queue는 나중에 오래된 센서 데이터를 처리할 때 사용될 수 있을 것 같다.
        self.y_Buffer = queue.Queue()
        # Buffer 초기화 작업 (n개의 데이터를 저장하는 buffer의 모든 데이터를 초기 측정값으로 채움, 필터링 전 일종의 보정을 하는 것)
        for _ in range(n):
            self.y_Buffer.put(y_initial_measure)
        # 마찬가지로 인스턴스 변수에 접근하기 위해 self. 를 붙여 멤버를 선언해 준다.
        self.n = n
        self.y_estimate = y_initial_measure

    # y_estimate를 구하는 실제 이동평균필터 방정식을 적용시키는 함수
    def estimate(self, y_measure):
        # 인스턴스 접근을 위한 멤버 선언
        self.y_measure = y_measure
        # 해당 시점의 Buffer에 가장 먼저 추가되었던 y_measure 값을 제거함과 동시에 반환하여 y_previous_measure에 할당한다.
        self.y_previous_measure = self.y_Buffer.get()
        # FIFO 방식의 데이터 처리로 인해 한 자리가 남는다. 그 자리에 새로운 y_measure 값을 넣어준다.
        self.y_Buffer.put(y_measure)
        # *** 고로 여기서 y_previous_measure는 buffer에서 가장 최근에 떨어져 나온, x_(k-u)에 해당하는 측정치이다. ***
        self.y_estimate = self.y_estimate + (self.y_measure - self.y_previous_measure) / self.n
        # 결국 이 메소드의 목적은 추정치 self.y_estimate를 계산하는 것이다.
        return self.y_estimate

# 스크립트 파일 구분
if __name__ == "__main__":
    # pandas.read_csv 메소드로 엑셀 데이터 불러오고
    signal = pd.read_csv("Data/example_Filter_3.csv")
    # dataFrame 초기화 해준다
    signal['y_estimate'] = np.zeros(len(signal))
    # y_estimate 인스턴스 생성
    y_estimate = MovingAverageFilter(signal.y_measure[0])

    # 행, 열 수 만큼 반복 (y_estimate의 계산값을 update한다.)
    for i, row in signal.iterrows():
        # MovingAverageFilter 클래스의 estimate 메소드를 사용하여 signal의 y_measure행 i열의 값으로 y_estimate를 계산한다.
        y_estimate.estimate(signal.y_measure[i])
        # 위에서 계산된 y_estimate를 새로운 행 i에 update 한다.
        signal.y_estimate[i] = y_estimate.y_estimate

    # 밑에는 AverageFilter와 동일하다
    print(signal)

    plt.figure('ex02_MovingAverageFilter')
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()