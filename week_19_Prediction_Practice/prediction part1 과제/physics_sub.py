import pickle
import numpy
import os
import glob
import matplotlib.pyplot as plt
import numpy as np
import sys
import copy
from scipy.stats import norm, multivariate_normal



Model_name = ["CV", "CA", "CTRV", "CTRA"]




class CV():
    def __init__(self, dt=0.1):

        """
        x : [x, y, vx, vy]
        """

        self.dt = dt
        self.A = np.array([[1,0,dt,0],
                           [0,1,0,dt],
                           [0,0,1,0],
                           [0,0,0,1]])

    def step(self, x):

        self.x = np.dot(self.A, x)

        return self.x

    def pred(self, x, t_pred):

        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)

class CA():
    def __init__(self,  dt=0.1):
        """
        x : [x, y, vx, vy, ax, ay]
        """

        self.dt = dt
        self.A = np.array([[1,0,dt,0,(dt**2)/2,0],
                           [0,1,0,0,0,0],
                           [0,0,1,0,dt,0],
                           [0,0,0,1,0,0],
                           [0,0,0,0,1,0],
                           [0,0,0,0,0,1]])

    def step(self, x):

        self.x = np.dot(self.A, x)

        if (x[2])*(self.x[2])<=0:
            self.x[2]=0
            self.x[4]=0

        if (x[3])*(self.x[3])<=0:
            self.x[3]=0
            self.x[5]=0



        return self.x

    def pred(self, x, t_pred):

        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)


class CTRV():
    def __init__(self, dt=0.1):

        """
        x : [x, y, v, theta, theta_rate]
        """

        self.dt = dt

    def step(self, x):

        if np.abs(x[4])>0.1:
            self.x = [x[0]+x[2]/x[4]*(np.sin(x[3]+x[4]*self.dt)-np.sin(x[3])),
                      x[1]+x[2]/x[4]*(-np.cos(x[3]+x[4]*self.dt)+np.cos(x[3])),
                      x[2],
                      x[3]+x[4]*self.dt,
                      x[4]]

        else:
            self.x = [x[0]+x[2]*np.cos(x[3])*self.dt,
                      x[1]+x[2]*np.sin(x[3])*self.dt,
                      x[2],
                      x[3]+x[4]*self.dt,
                      x[4]]

        return self.x


    def pred(self, x,  t_pred):

        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)

class CTRA():
    def __init__(self, dt=0.1):

        """
        x : [x, y, v, a, theta, theta_rate]
        """
        self.dt = dt

    def step(self, x):

        if np.abs(x[5])>0.1:
            self.x = [x[0]+x[2]/x[5]*(np.sin(x[4]+x[5]*self.dt)-
                                                     np.sin(x[4]))+
                      x[3]/(x[5]**2)*(np.cos(x[4]+x[5]*self.dt)+
                                                self.dt*x[5]*np.sin(x[4]+x[5]*self.dt)-
                                                np.cos(x[4])),
                      x[1]+x[2]/x[5]*(-np.cos(x[4]+x[5]*self.dt)+
                                                     np.cos(x[4]))+
                      x[3]/(x[5]**2)*(np.sin(x[4]+x[5]*self.dt)-
                                                self.dt*x[5]*np.cos(x[4]+x[5]*self.dt)-
                                                np.sin(x[4])),
                      x[2]+x[3]*self.dt,
                      x[3],
                      x[4]+x[5]*self.dt,
                      x[5]]

        else:
            self.x = [x[0]+x[2]*np.cos(x[4])*self.dt,
                      x[1]+x[2]*np.sin(x[4])*self.dt,
                      x[2]+x[3]*self.dt,
                      x[3],
                      x[4],
                      x[5]]

        return self.x

    def pred(self, x, t_pred):
        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)



def simulation(sample, file_idx, model_idx, rate):

    """
    plot area
    """
    clip_list = np.array([
    [[-23,-15],[-10,20]],
    [[10,55],[-10,10]],
    [[-26,-17],[30,42]],
    [[-25,0],[0,30]]])

    clip_val = clip_list[file_idx]

    """
    Data parsing
    """
    pose = sample['pose']
    vel = sample['vel']
    theta = sample['heading']
    map_coords = sample['map']

    """
    a와 yaw rate은 일단 v와 heading의 변화량의 평균으로 가정
    """

    a_cand = (vel[1:]-vel[0:-1])
    a = np.ones_like(vel)*np.mean(a_cand)*10

    theta_rate_cand = (theta[1:]-theta[0:-1])
    theta_rate = np.ones_like(theta)*np.mean(theta_rate_cand)*10


    fig = plt.figure(figsize=(8,8))


    for i in range(len(pose)):

        if model_idx==0:
            x = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i])]
            model = CV(0.1)

        elif model_idx==1:
            x = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i]),
            a[i]*np.cos(theta[i]), a[i]*np.sin(theta[i])]
            model = CA(0.1)

        elif model_idx==2:
            x = [pose[i,0], pose[i,1], vel[i], theta[i], theta_rate[i]]
            model = CTRV(0.1)

        else:
            x = [pose[i,0], pose[i,1], vel[i], a[i], theta[i], theta_rate[i]]
            model = CTRA(0.1)

        X = model.pred(x, t_pred = 1)

        """
        Plot map data
        """
        for k in range(len(map_coords)):
            plt.plot(map_coords[k,:,0], map_coords[k,:,1],color='k', alpha=0.4, linewidth=1.4)

        """
        Plot True trajectory
        """
        plt.plot(pose[:,0],pose[:,1],'ro--', markersize=10, alpha=0.4)

        """
        Plot Predicted trajectory
        """
        temp = plt.plot(X[:,0],X[:,1],'bv--',markersize=14, alpha=0.4)


        plt.xlim(clip_val[0])
        plt.ylim(clip_val[1])


        plt.pause(rate)
        plt.cla()

    plt.show()





def main():

    # file_list = sorted(glob.glob("./sample/*.pickle"))
    file_list = glob.glob("./sample/*.pickle")

    file_idx = 0
    model_idx = 0
    rate  = 0.1

    try:
        file_idx = int(sys.argv[1]) # sample file idx
    except:
        pass

    try:
        model_idx = int(sys.argv[2]) # model
    except:
        pass

    try:
        rate = float(sys.argv[3]) # plot speed
    except:
        pass

    print('sample file number ', file_idx)

    print('Test model : ', Model_name[model_idx])

    with open(file_list[file_idx], 'rb') as f:
        sample = pickle.load(f)

    simulation(sample,file_idx,model_idx, rate)


if __name__ == '__main__':
    main()
