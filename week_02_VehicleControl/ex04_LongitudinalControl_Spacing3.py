import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Long import VehicleModel_Long
from ex02_LongitudinalControl_Spacing1 import PID_Controller_ConstantSpace  # CSP
from ex03_LongitudinalControl_Spacing2 import PID_Controller_ConstantTimeGap  # CTP


if __name__ == "__main__":
    
    step_time = 0.1
    simulation_time = 80.0
    m = 500.0
    
    vx_ego = []
    vx_target = []
    x_space = []
    x_reference = []
    timegap = []
    time = []
    target_vehicle = VehicleModel_Long(step_time, m, 0.0, 30.0, 10.0)
    ego_vehicle = VehicleModel_Long(step_time, m, 0.5, 0.0, 10.0)
    controller = PID_Controller_ConstantSpace(step_time, target_vehicle.x, ego_vehicle.x)  # CSP
    # controller = PID_Controller_ConstantTimeGap(step_time, target_vehicle.x, ego_vehicle.x, ego_vehicle.vx)  # CTP
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        vx_ego.append(ego_vehicle.vx)
        vx_target.append(target_vehicle.vx)
        x_space.append(target_vehicle.x - ego_vehicle.x)
        x_reference.append(controller.constantSpace)  # CSP
        # x_reference.append(controller.space)  # CTP
        timegap.append((target_vehicle.x - ego_vehicle.x)/ego_vehicle.vx)
        controller.ControllerInput(target_vehicle.x, ego_vehicle.x)  # CSP
        # controller.ControllerInput(target_vehicle.x, ego_vehicle.x, ego_vehicle.vx)  # CTP
        ego_vehicle.update(controller.u)
        # 총 80초 주행 중 => 0~20sec: 등속 / 20~40: 가속/ 40~60: 감속 / 60~80: 등속 
        if (time[i]<20):
            a_t = 0.0
        elif (time[i]<40):
            a_t = 1.5
        elif (time[i]<60):
            a_t = -1.5
        else:
            a_t = 0.0
        target_vehicle.update(a_t)
        # print(ego_vehicle.ax)
        
        
    plt.figure(1)
    plt.plot(time, vx_ego,'r-',label = "ego_vx [m/s]")
    plt.plot(time, vx_target,'b-',label = "target_vx [m/s]")
    plt.xlabel('time [s]')
    plt.ylabel('Vx')
    plt.legend(loc="best")
    plt.title("CSP (Velocity)")
    plt.axis("equal")
    plt.grid(True)
    
    plt.figure(2)
    plt.plot(time, x_reference,'k-',label = "reference space [m]")
    plt.plot(time, x_space,'b-',label = "space [m]")
    plt.xlabel('time [s]')
    plt.ylabel('x')
    plt.legend(loc="best")
    plt.title("CSP (Space)")
    plt.axis("equal")
    plt.grid(True)
    
    # plt.figure(3)
    # plt.plot([0, time[-1]], [controller.timegap, controller.timegap],'k-',label = "reference timegap [s]")
    # plt.plot(time, timegap,'b-',label = "timegap [s]")
    # plt.xlabel('time [s]')
    # plt.ylabel('x')
    # plt.legend(loc="best")
    # plt.axis("equal")
    # plt.grid(True)    
    
    plt.show()

    