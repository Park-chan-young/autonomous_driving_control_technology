import numpy as np

class VehicleModel_Lat(object):
    def __init__(self, step_time, Vx, m=500, L=4, kv=0.005):
        self.dt = step_time
        self.m = m
        self.L = L
        self.kv = kv
        self.vx = Vx
        self.yawrate = 0
        self.Yaw = 0
        self.X = 0
        self.Y = 0

    # 횡방향 제어기는 delta, 속도가 input
    def update(self, delta, Vx):
        self.vx = Vx
        # 조향각 delta값을 약 -28.65 ~ 28.65 degree로 가져간다.
        self.delta = np.clip(delta,-0.5,0.5)
        # self.kv*self.vx**2 얘는 뭘까...?? (속도가 너무 커졌을 때를 대비하는 건가?)
        self.yawrate = self.vx/(self.L+self.kv*self.vx**2)*self.delta
        # Yaw_new = Yaw_old + delta_Yaw
        self.Yaw = self.Yaw + self.dt*self.yawrate
        # X_new = X_old + delta_X
        self.X = self.X + Vx*self.dt*np.cos(self.Yaw)
        # Y_new = Y_old + delta_Y
        self.Y = self.Y + Vx*self.dt*np.sin(self.Yaw)

