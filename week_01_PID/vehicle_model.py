import numpy as np

class VehicleModel(object):
    # step_time: delta t, R: 노이즈 표준편차, force_ratio: 입력 힘 비율(?), force_bias: 힘 편향(?)
    def __init__(self, step_time, R, force_ratio, force_bias, m=1.0):
        # Y: 상태변수
        self.Y  = np.array([[1.0], [0.0]])
        # A, B, C: 상태공간에서 사용되는 행렬
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0],[step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        # force ratio(입력 힘 비율): 운전자가 가하는 입력 힘과 실제 차량에 적용되는 힘 사이의 비율 (공기저항, 마찰 등으로 인한 손실)
        self.r_f = force_ratio
        self.R = R
        # force bias(힘 편향): 차량에 가해지는 외부 힘 이외의 추가적인 힘 요소 (보통 차량의 가속도나 속도 변화와 관련)
        self.bias = np.array([[0], [force_bias/m*step_time]])
        # y_measure의 초기값은 [1; 0]x[1 0]=[1] 즉, y0=1
        # numpy 행렬 곱 연산자 '@'
        self.y_measure = self.C @ self.Y
    
    def ControlInput(self, u):
        self.Y = self.A @ self.Y  +  self.B * u * self.r_f + self.bias
        # np.random.normal(0.0, self.R): 평균이 0.0, 표준편차가 R인 가우시안 분포에서 랜덤 값으 생성하여 위치 측정에 노이즈를 부여
        self.y_measure = self.C @ self.Y + np.random.normal(0.0, self.R)