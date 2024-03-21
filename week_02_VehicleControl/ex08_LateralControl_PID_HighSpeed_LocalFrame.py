import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local
from ex06_GlobalFrame2LocalFrame import PolynomialFitting
from ex06_GlobalFrame2LocalFrame import PolynomialValue


class PD_Controller(object):
        def __init__(self, step_time, Y_ref, Vx, L, R=1/0.019177, Kv=-0.1, P_Gain=3.0, D_Gain=0.1):
            self.step_time = step_time
            self.Y_ref = Y_ref
            self.Vx = Vx
            self.L = L
            self.R = R
            self.Kv = Kv
            self.P_Gain = P_Gain
            self.D_Gain = D_Gain
            self.prev_error = 0.0

        def ControllerInput(self, Y_ref, Y):
            self.Y_ref = Y_ref
            self.Y = Y
            self.error = self.Y_ref - self.Y
            P_term = self.error * self.P_Gain
            D_term = (self.error - self.prev_error) * self.D_Gain / self.step_time
            self.u = P_term + D_term + (self.L + self.Kv * (self.Vx) ** 2)/self.R
            self.prev_error = self.error



if __name__ == "__main__":
    step_time = 0.1
    # 속도가 6배 늘어남에 따라 시뮬레이션 시간을 1/6으로 줄였다.
    simulation_time = 30.0/6
    Vx = 20.0
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0-2*np.cos(X_ref/10)
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
    controller = PD_Controller(step_time, polynomialvalue.y[0][0], Vx, ego_vehicle.L)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        # 변환 할 글로벌 좌표 5개 생성
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X+5.0, 1.0)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        # 로컬 좌표로 변환
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        # 계산된 local 함수값 반환
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        # control input이 최소제곱해, 속도 ...?
        controller.ControllerInput(polynomialvalue.y[0][0], 0.0)
        # cotrol output은 동일하게 delta
        ego_vehicle.update(controller.u, Vx)

        print(ego_vehicle.X)

        
    plt.figure(1)
    plt.plot(X_ref, Y_ref,'k-',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.title("Lateral Control at high speed (Kv < 0: Over-steer)")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


