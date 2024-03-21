import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex05_LateralControl_Kinematic import PID_Controller_Kinematic
from ex06_GlobalFrame2LocalFrame import Global2Local
from ex06_GlobalFrame2LocalFrame import PolynomialFitting
from ex06_GlobalFrame2LocalFrame import PolynomialValue
    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    # 글로벌 기준 y=2-2cos(x/10) 그래프 (얘는 plotting 할 때 그릴 기준선, 결과 비교대상임)
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0-2*np.cos(X_ref/10)
    num_degree = 3
    num_point = 5
    # 로컬 x좌표 (실제론 5개만 쓰임)
    x_local = np.arange(0.0, 10.0, 0.5)

    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)

    # 글로벌 -> 로컬 변환 할 좌표 5개, 3차식 곡선
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    # 로컬 기준 변환 된 polynimialvalue.y를 ref.y로, 현재 y를 origin 0으로 초기화
    controller = PID_Controller_Kinematic(step_time, polynomialvalue.y[0][0], 0.0)
    
    for i in range(int(simulation_time/step_time)):
        # 시간에 대한 글로벌 좌표계 에서의 x, y 값을 받아온다.
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        # 현재 글로벌 x좌표로 부터 1씩 더해지는 값 5개를 반환
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X+5.0, 1.0)
        # 이 x에 대한  y=2-2cos(x/10) 식의 y값 반환 (마찬가지로 5개)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        # 5x2 형태의 matrix로 전치
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        # 변환시킬 point, 글로벌의 yaw, x, y 값 => 로컬 point반환
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        # 로컬 point => 계수 행렬 coeff 반환 (5x1 행렬)
        polynomialfit.fit(frameconverter.LocalPoints)
        # coeff, 로컬 x좌표 => fitting 된 로컬에서의 y 값 도출
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        # 도출된 로컬에서의 y값으로 PID control (Y_Reference = 로컬에서 x_local=0일 때의 y / Y=0) => delta 도출
        # 왜 y_local(0) 값을 레퍼런스로 사용할까?
        ## "글로벌에서는 y 값의 차이를 기반으로 조향각을 생성하기 까다롭다.
        ## 그래서 로컬에서 현재 차량의 y좌표를 0으로 두고 local_x가 0일 때의 local_y 값 만을 사용하여 delta를 도출하고자
        ## 이 모든 과정을 거친것이다."
        controller.ControllerInput(polynomialvalue.y[0][0], 0.0)
        # 제어기에서 나온 delta 값, 속도 => 글로벌에서의 X, Y 도출
        # 제어기에서 나온 delta를 기반으로 글로벌에서의 위치를 알 수있고 같은 과정을 반복하여 delta를 구할 수 있다.
        ego_vehicle.update(controller.u, Vx)

# <정리>
# 글로벌(X,Y,yaw) -[변환]-> 로컬좌표(X,Y) -[피팅]-> 로컬값(CTE=local_y(0)) -[제어]-> 조향각(delta) -[모델]-> 글로벌(X,Y,yaw)
        
    plt.figure(1)
    plt.plot(X_ref, Y_ref,'k-',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.title("Lateral Control (PID, Vx=3m/sec)")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


