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

class KalmanFilter():
    def __init__(self, x_dim, z_dim):

        self.Q = np.eye(x_dim)
        self.R = np.eye(z_dim)
        self.B = None
        self.P = np.eye(x_dim)
        self.A = np.eye(x_dim)
        self.H = np.zeros((z_dim,x_dim))

        self.x = np.zeros((x_dim,1))
        self.y = np.zeros((z_dim,1))

        self.K = np.zeros((x_dim, z_dim))
        self.S = np.zeros((z_dim, z_dim))

        self._I = np.eye(x_dim)

        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        self.SI = np.zeros((z_dim, z_dim))
        self.inv = np.linalg.inv

    def predict(self, u=None, B=None, A=None, Q=None):

        if B is None:
            B = self.B
        if A is None:
            A = self.A
        if Q is None:
            Q = self.Q


        if B is not None and u is not None:
            self.x = np.dot(A, self.x) + np.dot(B, u)
        else:
            self.x = np.dot(A, self.x)


        self.P = np.dot(np.dot(A, self.P), A.T) + Q


        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()


    def correction(self, z, R=None, H=None):

        if R is None:
            R = self.R

        if H is None:
            H = self.H

        self.y = z - np.dot(H, self.x)

        PHT = np.dot(self.P, H.T)

        self.S = np.dot(H, PHT) + R
        self.SI = self.inv(self.S)

        self.K = np.dot(PHT, self.SI)

        self.x = self.x + np.dot(self.K, self.y)

        I_KH = self._I - np.dot(self.K, H)

        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) +\
        np.dot(np.dot(self.K, R), self.K.T)
        

        self.x_post = self.x.copy()
        self.P_post = self.P.copy()



class Extended_KalmanFilter():
    def __init__(self, x_dim, z_dim):

        self.Q = np.eye(x_dim)
        self.R = np.eye(z_dim)
        self.B = None
        self.P = np.eye(x_dim)
        self.JA = None
        self.JH = None

        self.F = (lambda x:x)
        self.H = (lambda x:np.zeros(z_dim,1))

        self.x = np.zeros((x_dim,1))
        self.y = np.zeros((z_dim,1))

        self.K = np.zeros((x_dim, z_dim))
        self.S = np.zeros((z_dim, z_dim))

        self.x_dim = x_dim
        self.z_dim = z_dim

        self._I = np.eye(x_dim)

        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        self.SI = np.zeros((z_dim, z_dim))
        self.inv = np.linalg.inv

        self.likelihood = 1.0

    def predict(self, u=None, JA=None, F=None, Q=None):

        if Q is None:
            Q = self.Q

        # x = Fx + Bu
        if JA is None:
            if self.JA is None:
                JA_ = np.eye(self.x_dim)
            else:
                JA_ = self.JA(self.x)
        else:
            JA_ = JA(self.x)

        if F is None:
            F = self.F

        self.x = F(self.x)

        # P = FPF' + Q
        self.P = np.dot(np.dot(JA_, self.P), JA_.T) + Q

        # save prior
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()


    def correction(self, z, JH = None, H=None, R=None):

        if JH is None:
            if self.JH is None:
                JH_ = np.zeros((self.x_dim,self.z_dim))
            else:
                JH_ = self.JH(self.x)
        else:
            JH_ = JH(self.x)

        if H is None:
            H = self.H

        z_pred = H(self.x)

        if R is None:
            R = self.R

        self.y = z - z_pred

        PHT = np.dot(self.P, JH_.T)

        self.S = np.dot(JH_, PHT) + R
        self.SI = self.inv(self.S)

        self.K = np.dot(PHT, self.SI)

        self.x = self.x + np.dot(self.K, self.y)

        I_KH = self._I - np.dot(self.K, JH_)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(self.K, R), self.K.T)

        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        self.likelihood = multivariate_normal.pdf(self.y, np.zeros_like(self.y), self.S)



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

        self.H = np.array([[1,0,0,0],
                            [0,1,0,0],
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
        self.A = np.array([[1,0,dt,0,dt**2,0],
                           [0,1,0,dt,0,dt**2],
                           [0,0,1,0,dt,0],
                           [0,0,0,1,0,dt],
                           [0,0,0,0,1,0],
                           [0,0,0,0,0,1]])

        self.H = np.array([[1,0,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,0,0,0],
                            [0,0,0,1,0,0]])

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
            self.x = [x[0]+x[2]/x[4]*(np.sin(x[3]+x[4]*self.dt)-
                                                     np.sin(x[3])),
                      x[1]+x[2]/x[4]*(-np.cos(x[3]+x[4]*self.dt)+
                                                     np.cos(x[3])),
                      x[2],
                      x[3]+x[4]*self.dt,
                      x[4]]

        else:
            self.x = [x[0]+x[2]*np.cos(x[3])*self.dt,
                      x[1]+x[2]*np.sin(x[3])*self.dt,
                      x[2],
                      x[3],
                      x[4]]

        return self.x

    def H(self,x):

        return np.array([x[0],x[1],x[2],x[3]])

    def JA(self,x,dt = 0.1):
        px = x[0]
        py = x[1]
        v = x[2]
        yaw = x[3]
        r = x[4]

        if np.abs(r)>0.1:
            # upper
            JA_ = [[1, 0, (np.sin(yaw+r*dt)-np.sin(yaw))/r, v/r*(np.cos(yaw+r*dt)-np.cos(yaw)),
                     -v/(r**2)*(np.sin(yaw+r*dt)-np.sin(yaw))+v/r*(dt*np.cos(yaw+r*dt))],
                    [0, 1, (np.sin(yaw+r*dt)-np.sin(yaw))/r, v/r*(np.cos(yaw+r*dt)-np.cos(yaw)),
                     -v/(r**2)*(np.sin(yaw+r*dt)-np.sin(yaw))+v/r*(dt*np.cos(yaw+r*dt))],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, dt],
                    [0, 0, 0, 0, 1]]
        else:
            JA_ = [[1, 0 , np.cos(yaw)*dt, -v*np.sin(yaw)*dt ,0],
                   [0, 1 , np.sin(yaw)*dt, v*np.cos(yaw)*dt,0],
                   [0,0,1,0,0],
                   [0,0,0,1,dt],
                   [0,0,0,0,1]]

        return np.array(JA_)

    def JH(self,x,dt = 0.1):


        JH_ = [[1,0,0,0,0],
               [0,1,0,0,0],
               [0,0,1,0,0],
               [0,0,0,1,0]]

        return np.array(JH_)


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
                      x[2]/(x[5]**2)*(np.cos(x[4]+x[5]*self.dt)+
                                                self.dt*x[5]*np.sin(x[4]+x[5]*self.dt)-
                                                np.cos(x[4])),
                      x[1]+x[2]/x[5]*(-np.cos(x[4]+x[5]*self.dt)+
                                                     np.cos(x[4]))+
                      x[2]/(x[5]**2)*(np.sin(x[4]+x[5]*self.dt)-
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

    def H(self,x):

        return np.array([x[0],x[1],x[2],x[4]])

    def JA(self,x,dt = 0.1):

        px = x[0]
        py = x[1]
        v = x[2]
        a = x[3]
        yaw = x[4]
        r = x[5]


        # upper
        if np.abs(r)>0.1:
            JA_ = [[1,0,(np.sin(yaw+r*dt)-np.sin(yaw))/r,(-np.cos(yaw)+np.cos(yaw+r*dt)+r*dt*np.sin(yaw+r*dt))/r**2,
                    ((r*v+a*r*dt)*np.cos(yaw+r*dt)-a*np.sin(yaw+r*dt)-v*r*np.cos(yaw)+a*np.sin(yaw))/r**2,
                    -2/r**3*((r*v+a*r*dt)*np.sin(yaw+r*dt)+a*np.cos(yaw+r*dt)-v*r*np.sin(yaw)-a*np.cos(yaw))+
                    ((v+a*dt)*np.sin(yaw+r*dt)+dt*(r*v+a*r*dt)*np.cos(yaw+r*dt)-dt*a*np.sin(yaw+r*dt)-v*np.sin(yaw))/r**2],
                    [0,1,(-np.cos(yaw+r*dt)+np.cos(yaw))/r,(-np.sin(yaw)+np.sin(yaw+r*dt)-r*dt*np.cos(yaw+r*dt))/r**2,
                    ((r*v+a*r*dt)*np.sin(yaw+r*dt)+a*np.cos(yaw+r*dt)-v*r*np.sin(yaw)-a*np.cos(yaw))/r**2,
                    -2/r**3*((-r*v-a*r*dt)*np.cos(yaw+r*dt)+a*np.sin(yaw+r*dt)+v*r*np.cos(yaw)-a*np.sin(yaw))+
                    ((-v-a*dt)*np.cos(yaw+r*dt)+dt*(r*v+a*r*dt)*np.sin(yaw+r*dt)+a*dt*np.cos(yaw+r*dt)+v*np.cos(yaw))/r**2],
                    [0,0,1,dt,0,0],
                    [0,0,0,1,0,0],
                    [0,0,0,0,1,dt],
                    [0,0,0,0,0,1]]
        else:
            JA_ = [[1, 0 , np.cos(yaw)*dt, 1/2*np.cos(yaw)*dt**2,-(v+1/2*a*dt)*np.sin(yaw)*dt ,0],
                    [0, 1 , np.sin(yaw)*dt, 1/2*np.sin(yaw)*dt**2, (v+1/2*a*dt)*np.cos(yaw)*dt,0],
                    [0,0,1,dt,0,0],
                    [0,0,0,1,0,0],
                    [0,0,0,0,1,dt],
                    [0,0,0,0,0,1]]

        return np.array(JA_)

    def JH(self,x, dt = 0.1):
        px = x[0]
        py = x[1]
        v = x[2]
        a = x[3]
        yaw = x[4]
        r = x[5]

        # upper
        if np.abs(r)>0.1:

            JH_ = [[1,0,(np.sin(yaw+r*dt)-np.sin(yaw))/r,(-np.cos(yaw)+np.cos(yaw+r*dt)+r*dt*np.sin(yaw+r*dt))/r**2,
                    ((r*v+a*r*dt)*np.cos(yaw+r*dt)-a*np.sin(yaw+r*dt)-v*r*np.cos(yaw)+a*np.sin(yaw))/r**2,
                    -2/r**3*((r*v+a*r*dt)*np.sin(yaw+r*dt)+a*np.cos(yaw+r*dt)-v*r*np.sin(yaw)-a*np.cos(yaw))+
                    ((v+a*dt)*np.sin(yaw+r*dt)+dt*(r*v+a*r*dt)*np.cos(yaw+r*dt)-dt*a*np.sin(yaw+r*dt)-v*np.sin(yaw))/r**2],
                    [0,1,(-np.cos(yaw+r*dt)+np.cos(yaw))/r,(-np.sin(yaw)+np.sin(yaw+r*dt)-r*dt*np.cos(yaw+r*dt))/r**2,
                    ((r*v+a*r*dt)*np.sin(yaw+r*dt)+a*np.cos(yaw+r*dt)-v*r*np.sin(yaw)-a*np.cos(yaw))/r**2,
                    -2/r**3*((-r*v-a*r*dt)*np.cos(yaw+r*dt)+a*np.sin(yaw+r*dt)+v*r*np.cos(yaw)-a*np.sin(yaw))+
                    ((-v-a*dt)*np.cos(yaw+r*dt)+dt*(r*v+a*r*dt)*np.sin(yaw+r*dt)+a*dt*np.cos(yaw+r*dt)+v*np.cos(yaw))/r**2],
                    [0,0,1,dt,0,0],
                    [0,0,0,0,1,dt]]

        else:
            JH_ = [[1, 0 , np.cos(yaw)*dt, 1/2*np.cos(yaw)*dt**2,-(v+1/2*a*dt)*np.sin(yaw)*dt ,0],
                    [0, 1 , np.sin(yaw)*dt, 1/2*np.sin(yaw)*dt**2, (v+1/2*a*dt)*np.cos(yaw)*dt,0],
                    [0,0,1,dt,0,0],
                    [0,0,0,0,1,dt]]

        return np.array(JH_)

    def pred(self, x, t_pred):
        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)

def plot_result(pose, vel, a, theta, theta_rate, X, model_idx):
    plt.figure(1, figsize=(10,10))

    if model_idx==0:
        plt.subplot(3,2,1)
        plt.plot(pose[:,0])
        plt.plot(X[:,0])

        plt.subplot(3,2,2)
        plt.plot(pose[:,1])
        plt.plot(X[:,1])

        plt.subplot(3,2,3)
        plt.plot(vel*np.cos(theta))
        plt.plot(X[:,2])

        plt.subplot(3,2,4)
        plt.plot(vel*np.sin(theta))
        plt.plot(X[:,3])


    elif model_idx==1:
        plt.subplot(3,2,1)
        plt.plot(pose[:,0])
        plt.plot(X[:,0])

        plt.subplot(3,2,2)
        plt.plot(pose[:,1])
        plt.plot(X[:,1])

        plt.subplot(3,2,3)
        plt.plot(vel*np.cos(theta))
        plt.plot(X[:,2])

        plt.subplot(3,2,4)
        plt.plot(vel*np.sin(theta))
        plt.plot(X[:,3])

        plt.subplot(3,2,5)
        plt.plot(a*np.cos(theta))
        plt.plot(X[:,4])

        plt.subplot(3,2,6)
        plt.plot(a*np.sin(theta))
        plt.plot(X[:,5])

    elif model_idx==1:
        plt.subplot(3,2,1)
        plt.plot(pose[:,0])
        plt.plot(X[:,0])

        plt.subplot(3,2,2)
        plt.plot(pose[:,1])
        plt.plot(X[:,1])

        plt.subplot(3,2,3)
        plt.plot(vel)
        plt.plot(X[:,2])

        plt.subplot(3,2,5)
        plt.plot(theta)
        plt.plot(X[:,3])

        plt.subplot(3,2,6)
        plt.plot(theta_rate)
        plt.plot(X[:,4])

    else:
        plt.subplot(3,2,1)
        plt.plot(pose[:,0])
        plt.plot(X[:,0])

        plt.subplot(3,2,2)
        plt.plot(pose[:,1])
        plt.plot(X[:,1])

        plt.subplot(3,2,3)
        plt.plot(vel)
        plt.plot(X[:,2])

        plt.subplot(3,2,4)
        plt.plot(a)
        plt.plot(X[:,3])

        plt.subplot(3,2,5)
        plt.plot(theta)
        plt.plot(X[:,4])

        plt.subplot(3,2,6)
        plt.plot(theta_rate)
        plt.plot(X[:,5])

    plt.show()

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
    a와 yaw rate은 v와 heading의 변화량 (비교용)
    """

    a_cand = (vel[1:]-vel[0:-1])*10
    a = np.insert(a_cand, 0, a_cand[0], axis=0)

    theta_rate_cand = (theta[1:]-theta[0:-1])*10
    theta_rate = np.insert(theta_rate_cand, 0, theta_rate_cand[0], axis=0)


    """
    Kalman Filter initialize
    """

    if model_idx==0:
        x_init = [pose[0,0], pose[0,1], vel[0]*np.cos(theta[0]), vel[0]*np.sin(theta[0])]

        model = CV(0.1)
        kf = KalmanFilter(4,4)
        kf.A = model.A
        kf.H = model.H
        kf.x = x_init


    elif model_idx==1:
        x_init = [pose[0,0], pose[0,1], vel[0]*np.cos(theta[0]), vel[0]*np.sin(theta[0]),
        a[0]*np.cos(theta[0]), a[0]*np.sin(theta[0])]

        model = CA(0.1)
        kf = KalmanFilter(6,4)
        kf.A = model.A
        kf.H = model.H
        kf.x = x_init


    elif model_idx==2:
        x_init = [pose[0,0], pose[0,1], vel[0], theta[0], 0]
        model = CTRV(0.1)

        kf = Extended_KalmanFilter(5,4)

        kf.F = model.step
        kf.JA = model.JA
        kf.H = model.H
        kf.JH = model.JH

        kf.x = x_init


    else:
        x_init = [pose[0,0], pose[0,1], vel[0], 0, theta[0], 0]
        model = CTRA(0.1)

        kf = Extended_KalmanFilter(6,4)

        kf.F = model.step
        kf.JA = model.JA
        kf.H = model.H
        kf.JH = model.JH

        kf.x = x_init


    X = [x_init]


    fig = plt.figure(0, figsize=(8,8))


    """
    #####################################################
    To do : Q , R 값을 바꿔가면서 결과에 어떤 영향을 미치는지 고찰
    #####################################################
    """

    for i in range(len(pose)):

        if model_idx==0:
            x = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i])]
            z = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i])]

            kf.predict(Q=np.diag([1,1,1,1]))

        elif model_idx==1:
            x = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i]),
            a[i]*np.cos(theta[i]), a[i]*np.sin(theta[i])]
            z = [pose[i,0], pose[i,1], vel[i]*np.cos(theta[i]), vel[i]*np.sin(theta[i])]

            kf.predict(Q=np.diag([1,1,1,10,1,10]))

        elif model_idx==2:
            x = [pose[i,0], pose[i,1], vel[i], theta[i], theta_rate[i]]
            z = [pose[i,0], pose[i,1], vel[i], theta[i]]

            kf.predict(Q=np.diag([1,1,1,1,10]))

        else:
            x = [pose[i,0], pose[i,1], vel[i], a[i], theta[i], theta_rate[i]]
            z = [pose[i,0], pose[i,1], vel[i], theta[i]]

            # kf.predict(Q=np.diag([1,1,1,10,1,10]))
            kf.predict(Q=np.diag([0.001,0.001,0.01,0.01,0.1,0.1]))


        # kf.correction(z=z, R=np.diag([1,1,1,1]))
        kf.correction(z=z, R=np.diag([1,1,1,1]))


        model_kf = copy.deepcopy(model)

        XX = model_kf.pred(kf.x, t_pred = 1)
        YY = model.pred(x, t_pred = 1)

        X.append(kf.x)

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
        Plot Kalman filtered trajectory
        """
        plt.plot(XX[:,0],XX[:,1],'bv--',markersize=14, alpha=0.3)

        """
        Plot sensor-based trajectory
        """
        plt.plot(YY[:,0],YY[:,1],'gs--', markersize=10, alpha=0.3)


        plt.xlim(clip_val[0])
        plt.ylim(clip_val[1])


        plt.pause(rate)
        plt.cla()

    plt.show()


    plot_result(pose, vel, a, theta, theta_rate, np.array(X), model_idx)


def main():

    file_list = glob.glob("./sample/*.pickle")

    file_idx = 3
    model_idx = 3
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
