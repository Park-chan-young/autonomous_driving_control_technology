import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 필터 클래스 생성
class AverageFilter:
    # 클래스 초기화
    def __init__(self, y_initial_measure):
        # 추정치 초기값 = 측정치 초기값
        self.y_estimate = y_initial_measure
        # 계산할 측정값의 개수
        self.k = 1
        
    # *** 실제 평균필터 방정식을 적용시키는 함수 ***
    def estimate(self, y_measure):
        # 측정치 (csv 파일에서 받아올 값)
        self.y_measure = y_measure
        # Q)메서드 내에서 계산할 때 멤버 앞에는 무조건 self. 를 붙여야 하는건가...?
        # A)self. 는 인스턴스를 나타내는 것이다. 메서드 내에서 self.를 사용해야 인스턴스 변수에 접근할 수 있기 때문에 꼭 써야한다.
        self.y_estimate = (self.k-1) * self.y_estimate / self.k + self.y_measure / self.k
        self.k += 1


# if __name__ == "__main__": 이게 스크립트 파일을 구분하기 위한 표시라고 한다.
# 현재 모듈에서 직접 실행 시켜야만 작동하는 블록(?)이다. 다른 모듈에서 import 해서 사용할 때 작동하지 않는 부분이다.
if __name__ == "__main__":
    # pandas 라이브러리의 scv를 읽는 함수를 사용하여 signal에 2차원 dataFrame 형식으로 데이터를 저장한다. (Matrix)
    signal = pd.read_csv("Data/example_Filter_1.csv")
    # 실행 전에 y_estimate 열의 성분들을 0으로 초기화 시켜주는 작업이다. (없어도 되는데 공부삼아 넣었다.)
    signal['y_estimate'] = np.zeros(len(signal))

    # y_estimate이라는 인스턴스 생성 (초기 x0 저장됨)
    y_estimate = AverageFilter(signal.y_measure[0])

    # 행, 열 수 만큼 반복 (y_estimate의 계산값을 update한다.)
    for i, row in signal.iterrows():
        # MovingAverageFilter 클래스의 estimate 메소드를 사용하여 signal의 y_measure행 i열의 값으로 y_estimate를 계산한다.
        y_estimate.estimate(signal.y_measure[i])
        # 위에서 계산된 y_estimate를 새로운 행 i에 update 한다.
        signal.y_estimate[i] = y_estimate.y_estimate

    # 모두 계산 된 siganal의 dataFrame을 한번 살펴본다.
    print(signal)

    # MatLAB Plot과 매우 유사하다. (아마 거기서 가져온 거 아닐까 하는 생각..?)
    # figure 제목
    plt.figure('ex01_AverageFilter')
    # 플롯팅 할 (x좌표, y좌표, 선종류, 데이터 명)을 입력한다.
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    # x, y 좌표계 이름을 설정한다.
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    # legend(범례)가 그래프 상 최적의 위치에 배치되도록 자동으로 설정하라는 의미이다.
    plt.legend(loc="best")
    # x, y축의 크기를 동일하게 설정하라는 의미이다.
    plt.axis("equal")
    # grid on
    plt.grid(True)
    plt.show()

