import numpy as np

# 현재 차량의 x, vx, ax 를 반환하는 class
class VehicleModel_Long(object):
    def __init__(self, step_time, m, Ca, x_init, Vx_init):
        self.x = x_init
        self.vx = Vx_init
        self.ax = 0.0
        self.yawRate = 0.0
        self.dt = step_time
        self.theta = 0.0
        self.m = m
        self.g = 9.81
        self.C = Ca/m
        self.delta = 0.0
        
    # 종방향 제어기는 가속도가 input
    def update(self, a_x):
        # x: 현재 위치, 계산된 vx와 ax를 적분해 기존 x에 더함으로서 현재 위치를 반환한다.
        self.x = self.x + self.dt*self.vx + (self.dt**2)*self.ax/2
        # vx: 현재 속도, 계산된 ax를 적분해 기존 vx에 더함으로서 현재 속도를 반환한다.
        self.vx = self.vx + self.dt*(self.ax)
        # np.clip(): 원소들을 지정된 범위 안으로 자르는 함수
        # np.clip(a,a_min,a_max) => 배열 a의 원소를 a_min 보다 작은 값은 a_min으로 대체, a_max보다 크면 a_max로 대체하여 배열을 반환한다.
        # m*ax = tractionForce - dragForce - mg*sin(theta) 에서 Ca를 공기저항 계수로 놓은 것 같다.
        self.ax = np.clip(a_x - self.C*(self.vx**2)-self.g*np.sin(self.theta), -2.0, 2.0)

        