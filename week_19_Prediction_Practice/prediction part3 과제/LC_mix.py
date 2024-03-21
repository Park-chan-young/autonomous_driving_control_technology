import pickle
import numpy
import os
import glob
import matplotlib.pyplot as plt
import numpy as np
import sys
import copy
from scipy.stats import norm, multivariate_normal


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



def generate_LC_profile(target_t):

    """
    Generate LC profile (v, x, y)
    """
    w = 3.4
    v_dec = lambda x: w/target_t*np.sin(np.pi/2*x/(w/2))

    s=w/2
    V = []

    while(w-s>0.1):
        v_ = v_dec(s)
        s +=v_*0.1
        V.append(v_)

    V = np.array(V)
    v_profile = np.concatenate([V[::-1], V], axis=0)

    sx=0
    Sy = [0]
    Sx = [0]
    vx = 10
    s = 0

    for i in range(len(v_profile)):
        sx += vx*0.1
        Sx.append(sx)
        s +=v_profile[i]*0.1
        # print(v_profile[i])
        Sy.append(s)

    traj = np.stack([Sx, Sy], -1)

    return v_profile, traj

def simulation(v_lc_profile, traj, rate):

    """
    Prediction parameter define
    """
    w = 3.4
    vx = 10
    dt=0.1
    T = 4
    future_step = int(T/dt)


    """
    Bayesian Network function and parameter define
    """

    p_lc = lambda x : multivariate_normal.pdf(x, w/2,w/4)
    p_lk = lambda x : (multivariate_normal.pdf(x,0, w/4)+multivariate_normal.pdf(x,w, w/4))

    p_lcv = lambda x, v : multivariate_normal.pdf(v, (-(2/w)**2*1.133*(x-w/2)**2+1.133),0.4)
    p_lkv = lambda x, v: (multivariate_normal.pdf(v, (-(2/w)**2*1.133*(x)**2),0.4)+\
                     multivariate_normal.pdf(v, (2/w)**2*1.133*(x-w)**2,0.4))

    p_lkv2 = lambda x, v: (multivariate_normal.pdf(v, (2/w)**2*1.133*(x-w)**2,0.4))

    P_LC = []
    P_LK = []

    P_LCV = []
    P_LKV = []
    P_LKV2 = []


    """
    model mix function
    """

    ft = lambda t : (1-1/(1+np.exp(-3*(t-1.2))))
    """
    #####################################################
    To do : mix function을 바꿔가면서 결과 확인
    #####################################################
    """

    """
    Kalman Filter for physics model
    """

    dx = traj[1:,0]-traj[:-1,0]
    dy = traj[1:,1]-traj[:-1,1]

    yaw = np.arctan2(dy, dx)
    yaw = np.concatenate([[0],yaw])

    filters = Extended_KalmanFilter(6,4)
    model = CTRA()

    Q =[0.1,0.1,0.1,0.1,0.001,0.01]

    traj_v = np.sqrt(v_lc_profile**2+vx**2)
    x = np.array([traj[0,0], traj[0,1], traj_v[0], 0, yaw[0],0])

    filters.F=model.step
    filters.H=model.H
    filters.JA=model.JA
    filters.JH=model.JH
    filters.Q = np.diag(Q)
    filters.R = np.diag([0.1,0.1,0.1,0.01])
    filters.x = x


    for i in range(1,len(v_lc_profile)-1):
        plt.subplot(2,1,1)

        ## Lane plotting ###
        plt.plot([-20,80],[-w/2,-w/2],'k-')

        plt.plot([-20,80],[w/2,w/2],'k-')
        plt.plot([-20,80],[3/2*w,3/2*w],'k-')

        plt.plot([-20,80],[-w/2,-w/2],'k-')

        plt.plot([-20,80],[w/2,w/2],'k-')
        plt.plot([-20,80],[3/2*w,3/2*w],'k-')


        """
        physics-based prediction
        """

        z = np.array([traj[i,0],traj[i,1], traj_v[i],yaw[i]])

        filters.predict()
        filters.correction(z)
        traj_phy = model.pred(filters.x, t_pred = T)


        Xp = traj_phy[:,0]
        Yp = traj_phy[:,1]

        """
        maneuver-based prediction
        """
        sigma_dela = 0.02
        sx = traj[i,0]
        sy = traj[i,1]

        P_LC.append(p_lc(sy))
        P_LK.append(p_lk(sy))
        P_LCV.append(p_lcv(sy,v_lc_profile[i]))
        P_LKV.append(p_lkv(sy,v_lc_profile[i]))

        p_lc_f = np.array(P_LCV)/(np.array(P_LCV)+np.array(P_LKV))*np.array(P_LC)
        p_lk_f =np.array(P_LKV)/(np.array(P_LCV)+np.array(P_LKV))*np.array(P_LK)


        Xm = [traj[i,0]]
        Ym = [traj[i,1]]


        if p_lc_f[-1] > p_lk_f[-1]:

            """
            p_LC > p_LK  ---> Lane change maneuver
            """
            lr_k = w*np.pi/(2*np.tan(yaw[i]))*np.cos(np.arcsin(2*traj[i,1]/w-1))
            del_x = (1/2+1/np.pi*np.arcsin(2*traj[i,1]/w-1))*lr_k

            d_left = lr_k-del_x

            s_move=0

            for t in range(future_step):
                if (s_move>d_left):

                    s_move += vx*dt
                    Ym.append(w)
                    Xm.append(traj[i,0]+s_move)
                else:
                    s_move += vx*dt
                    x = del_x+s_move

                    wa = np.random.normal(0,sigma_dela,1)
                    y = w/2*np.sin(np.pi/lr_k*x-np.pi/2)+w/2 + wa

                    Xm.append(traj[i,0]+s_move)
                    Ym.append(y)

        else:
            """
            p_LC < p_LK  ---> Follow Road maneuver
            """
            alpha=0.4
            s_move = 0
            for t in range(future_step):
                s_move +=vx*dt

                if traj[i,1]>w/2:
                    u = w
                else:
                    u = 0

                x_lat = u+(traj[i,1]-u)*np.exp(-alpha * dt*t)

                Ym.append(x_lat)
                Xm.append(traj[i,0]+s_move)



        """
        model mixing based on mix function ft
        """

        X = []
        Y = []

        for t in range(future_step):
            wp = ft(t*dt)
            wm = 1-wp

            X.append(wp*Xp[t]+wm*Xm[t])
            Y.append(wp*Yp[t]+wm*Ym[t])



        """
        Plot trajectory
        """

        plt.plot(traj[:,0],traj[:,1], 'rx-')
        plt.plot(Xp[0:40:4],Yp[0:40:4],'mv-')
        plt.plot(Xm[0:40:4],Ym[0:40:4],'gv-')
        plt.plot(X[0:40:4],Y[0:40:4],'bo-')

        plt.xlim([-5,80])


        """
        LC & LK probability
        """
        plt.subplot(2,1,2)

        plt.plot(traj[0:i,0], p_lc_f/(p_lc_f+p_lk_f), 'b-')
        plt.plot(traj[0:i,0], p_lk_f/(p_lc_f+p_lk_f), 'r-')
        plt.legend(["LC","LK"])
        plt.xlim([-10,70])

        plt.pause(rate)

        plt.subplot(2,1,1)
        plt.cla()

        plt.subplot(2,1,2)
        plt.cla()

    plt.show()


def main():


    target_t = 3 # time to cross the lane, used to genetate v_lc_profile, traj
    rate  = 0.1

    try:
        rate = float(sys.argv[1]) # plot speed
    except:
        pass

    v_lc_profile, traj = generate_LC_profile(target_t)

    simulation(v_lc_profile, traj, rate)




if __name__ == '__main__':
    main()
