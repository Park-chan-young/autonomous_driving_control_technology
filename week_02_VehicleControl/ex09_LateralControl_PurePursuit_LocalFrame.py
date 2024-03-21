import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local
from ex06_GlobalFrame2LocalFrame import PolynomialFitting
from ex06_GlobalFrame2LocalFrame import PolynomialValue

# ld=k*Vx 의 형태로 설계하고 싶었으나, ld에 있는 desired-path의 좌표를 구하는 방법을 몰라,
# 로컬 origin 기준으로 속도에 비례한 x값을 Xp=k*Vx 라 두고, ld를 상수로 둬서 waypoint의 좌표를 (Xp, Yp)로 설정했다.
# 근데 뭐가 이상하다...
class PurePursuit(object):
    def __init__(self, Vx, L, K=1.0):
        self.Vx = Vx
        self.L = L
        self.K = K # (K > 1)
        self.ld = K*Vx

    def ControllerInput(self, X, Y):
        self.X = X
        self.Y = Y
        self.Xp = self.X * self.K
        self.dx = self.Xp - self.X
        self.dy = np.sqrt(self.ld - self.dx ** 2)
        self.delta = np.arctan2(2*self.dy*self.L, self.ld**2)
        
    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0/6
    Vx = 20.0
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0 - 2 * np.cos(X_ref/10)
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)
    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)

    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    # 속도와 wheelbase는 변하지 않는 상수
    controller = PurePursuit(Vx, ego_vehicle.L)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X+5.0, 1.0)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        # fitting된 local의 x, y, 좌표로 계산 할 것임 => polynoimialvalue에서 x, y 가져옴
        controller.ControllerInput(polynomialvalue.x[0][0], polynomialvalue.y[0][0])
        ego_vehicle.update(controller.delta, Vx)

        print(controller.delta)
        print(ego_vehicle.X)
        print(ego_vehicle.Y)
        print("--------------------")

        
    plt.figure(1)
    plt.plot(X_ref, Y_ref,'k-',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


