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




"""
Cartesian <-> Frenet
"""

def next_waypoint(x, y, mapx, mapy):
    closest_wp = get_closest_waypoints(x, y, mapx, mapy)

    if closest_wp>=len(mapx)-1:
        closest_wp -=1
    map_vec = [mapx[closest_wp + 1] - mapx[closest_wp], mapy[closest_wp + 1] - mapy[closest_wp]]
    ego_vec = [x - mapx[closest_wp], y - mapy[closest_wp]]

    direction  = np.sign(np.dot(map_vec, ego_vec))

    if direction >= 0:
        next_wp = closest_wp + 1
    else:
        next_wp = closest_wp

    return next_wp


def get_closest_waypoints(x, y, mapx, mapy):
    min_len = 1e10
    closeset_wp = 0

    for i in range(len(mapx)):
        _mapx = mapx[i]
        _mapy = mapy[i]
        dist = get_dist(x, y, _mapx, _mapy)

        if dist < min_len:
            min_len = dist
            closest_wp = i

    return closest_wp


def get_dist(x, y, _x, _y):
    return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet(x, y, mapx, mapy):
    next_wp = next_waypoint(x, y, mapx, mapy)
    prev_wp = next_wp -1

    n_x = mapx[next_wp] - mapx[prev_wp]
    n_y = mapy[next_wp] - mapy[prev_wp]
    x_x = x - mapx[prev_wp]
    x_y = y - mapy[prev_wp]

    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
    proj_x = proj_norm*n_x
    proj_y = proj_norm*n_y

    #-------- get frenet d
    frenet_d = get_dist(x_x,x_y,proj_x,proj_y)

    ego_vec = [x-mapx[prev_wp], y-mapy[prev_wp], 0];
    map_vec = [n_x, n_y, 0];
    d_cross = np.cross(ego_vec,map_vec)
    if d_cross[-1] > 0:
        frenet_d = -frenet_d;

    #-------- get frenet s
    frenet_s = 0;
    for i in range(prev_wp):
        frenet_s = frenet_s + get_dist(mapx[i],mapy[i],mapx[i+1],mapy[i+1]);

    frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y);

    return frenet_s, frenet_d

def get_cartesian(s, d, mapx, mapy, maps):
    prev_wp = 0

#     s = np.mod(s, maps[-2])

    while(s > maps[prev_wp+1]) and (prev_wp < len(maps)-2):
        prev_wp = prev_wp + 1

    next_wp = np.mod(prev_wp+1,len(mapx))

    dx = (mapx[next_wp]-mapx[prev_wp])
    dy = (mapy[next_wp]-mapy[prev_wp])

    heading = np.arctan2(dy, dx) # [rad]

    # the x,y,s along the segment
    seg_s = s - maps[prev_wp];

    seg_x = mapx[prev_wp] + seg_s*np.cos(heading);
    seg_y = mapy[prev_wp] + seg_s*np.sin(heading);

    perp_heading = heading + 90 * np.pi/180;
    x = seg_x + d*np.cos(perp_heading);
    y = seg_y + d*np.sin(perp_heading);

    return x, y, heading

"""
Cartesian <-> Frenet End
"""

def make_R(route):

    R = []

    for i in range(3,len(route)):
        p1 = route[i-3]
        p2 = route[i-2]
        p3 = route[i-1]
        p4 = route[i]

        A = np.array([[p1[0],p1[1],1],
                     [p2[0],p2[1],1],
                     [p3[0],p3[1],1],
                     [p4[0],p4[1],1]])

        B = np.array([-(p1[0]**2+p1[1]**2),
                     -(p2[0]**2+p2[1]**2),
                     -(p3[0]**2+p3[1]**2),
                      -(p4[0]**2+p4[1]**2)])

        x = np.dot(np.linalg.inv(np.dot(A.T,A)),np.dot(A.T,B))

        r = np.sqrt(x[0]**2+x[1]**2-4*x[2])/2


        R.append(r)

    return np.concatenate([np.array([R[0] for i in range(3)]), R])

def find_a(lon, t, maps, R, target_a):

    s = lon[0].copy()
    v = lon[1].copy()
    a = lon[2].copy()

    wr = np.random.normal(0,1,1)

    min_idx = np.argmin(np.abs(maps-(s+t*v)))

    if maps[min_idx]>(s+t*v):
        min_idx -=1

    r = (R[min_idx]*(maps[min_idx+1]-(s+t*v))+R[min_idx+1]*((s+t*v)-maps[min_idx]))\
        /(maps[min_idx+1]-maps[min_idx])+wr

    v_tar = np.sqrt(target_a*r)

    a_tar = (v_tar**2-v**2)/(2*t*v)


    min_idx_cur = np.argmin(np.abs(maps-(s)))

    if maps[min_idx_cur]>(s):
        min_idx_cur -=1

    r_cur = (R[min_idx_cur]*(maps[min_idx_cur+1]-(s))+R[min_idx_cur+1]*((s)-maps[min_idx_cur]))\
            /(maps[min_idx_cur+1]-maps[min_idx_cur])+wr

    v_tar_cur = np.sqrt(target_a*r_cur)

    a_tar_cur = (v_tar_cur**2-v**2)/(0.1)

    a_tar_ = np.clip(np.min([a_tar, a_tar_cur, a]), -9.81,9.81)


    return a_tar_

def simulation(data, file_name, rate):

    """
    Data parsing
    """
    traj_veh = data["traj"]
    traj_veh_vel = data["traj_vel"]
    traj_front= data["traj_front"]
    traj_front_vel= data["traj_front_vel"]
    local_map= data["local_map"]
    local_map_color= data["local_map_color"]
    lane_agent_mask= data["lane_agent_mask"]
    route = data["route"] # Ego path

    """
    plot area
    """
    clip_list = {
        'fr' : np.array([[-30,20],[-30,15]]),
        'fv' : np.array([[-30,0],[-15,15]]),
        'st' : np.array([[0,30],[-40,0]]),
        'tu' : np.array([[-10,40],[-20,60]])
    }

    clip_val = clip_list[file_name]

    """
    make maps
    """

    r_diff = route[1:,:]-route[:-1,:]
    b = np.sqrt(r_diff[:,0]**2 + r_diff[:,1]**2)
    c = np.cumsum(b, axis=-1)

    lane_maps = np.concatenate([[0], c], axis=-1)

    """
    make R
    """
    lane_R = make_R(route)

    """
    Define parameter for maneuver based prediction
    """

    T = 4
    dt = 0.1
    a_max=0.5
    sigma_dela = a_max*dt
    sigma_limit = 0.05
    h = 1.0
    lamb = 0.8

    alpha = 0.4
    A = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, 0, 1]])
    B = np.array([0.5*dt**2, dt, 1])


    h_future = int(T/dt)


    """
    종방향 kalman filter for estimating acceleration
    """
    H = np.array([[1,0,0],[0,1,0]])

    kf = KalmanFilter(3,2)
    kf.A = A
    kf.H = H

    s, d = get_frenet(traj_veh[0,0],
                        traj_veh[0,1],
                        route[:,0],route[:,1])

    x = [s,traj_veh_vel[0], 0]
    kf.x = x

    # 전방 차량이 있는 경우
    if len(traj_front)>0:
        kf2 = KalmanFilter(3,2)
        kf2.A = A
        kf2.H = H

        s2, d2 = get_frenet(traj_front[0,0],
                            traj_front[0,1],
                            route[:,0],route[:,1])

        x2 = [s2,traj_front_vel[0],0]
        kf2.x=x2


    for k in range(1, len(traj_veh)):
        for j in range(np.sum(lane_agent_mask)):

            plt.plot(local_map[j,:,0],local_map[j,:,1],'-',color=local_map_color[j]*0.9, alpha=0.3,
                    linewidth=1.5)

        plt.plot(route[:,0], route[:,1], color='y', alpha=0.3, linewidth=4)


        plt.plot(traj_veh[0:-1:5,0],traj_veh[0:-1:5,1], 'bo-', alpha=0.6)

        if len(traj_front)>0:
            plt.plot(traj_front[0:-1:5,0],traj_front[0:-1:5,1], 'go-', alpha=0.6)

        """
        acceleration estimating
        """
        s, d = get_frenet(traj_veh[k,0],
                        traj_veh[k,1],
                        route[:,0],route[:,1])


        z= [s, traj_veh_vel[k]]

        kf.predict(Q=np.diag([1,1,4]))
        kf.correction(z=z, R=np.diag([1,1]))

        if len(traj_front)>0:
            s2, d2 = get_frenet(traj_front[k,0],
                                traj_front[k,1],
                                route[:,0],route[:,1])
            z2= [s2, traj_front_vel[k]]

            kf2.predict(Q=np.diag([1,1,4]))
            kf2.correction(z=z2, R=np.diag([1,1]))



        x_lon = kf.x.copy()
        x_lon_temp = kf.x.copy()
        x_lat = d

        if len(traj_front)>0:
            x_lon2 = kf2.x.copy()

        x_lons = np.zeros((h_future,1))
        x_lons_temp = np.zeros((h_future,1))
        sigma_lons = np.zeros((h_future,1))
        x_lats = np.zeros(( h_future,1))
        sigma_lats = np.zeros((h_future,1))

        for t in range(h_future):

            """
            #########################################
            Longitudinal movement start
            #########################################
            """

            sigma_lons[t] = 0.5*(dt*t)**2 * sigma_dela
            wa = np.random.normal(0,sigma_dela,1)


            if len(traj_front)>0:

                vs_target = (x_lon2[1]+x_lon2[2]*dt*t)
                s_target = x_lon2[0]+x_lon2[1]*dt*t+1/2*x_lon2[2]*dt**2*t
                delta = -s_target+x_lon[0]+h*x_lon[1]*dt+7

                e_dot = x_lon[1] - vs_target

                x_lon[2] = np.min([np.clip(-1/h*(e_dot+lamb*delta), -5,5),x_lon[2]])
                x_lon[2] = np.clip(-1/h*(e_dot+lamb*delta), -5,5)


            s_target = x_lon[0]+h*x_lon[1]

            if file_name == 'fr':
                # fr case

                a_target = x_lon[2]
            elif file_name == 'tu':
                # tu case

                a_target = find_a(x_lon, h, lane_maps, lane_R, target_a=3)

            elif file_name == 'fv':
                # fv case


                """
                To Do

                """
                delta = x_lon[0] - s_target
                e_dot = x_lon[1] - vs_target
                a_target = -(e_dot + lamb*delta)/h


            elif file_name == 'st':
                # st case

                """
                To Do

                """
                s_target = 15
                vs_target = 0
                delta = x_lon[0] - s_target
                e_dot = x_lon[1] - vs_target
                a_target = -(e_dot + lamb*delta)/h
                

            x_lon[2] = a_target


            """
            prevent moving reverse direction
            """
            if (x_lon[2]*dt + wa*dt + x_lon[1])<=0 or x_lon[1]<0.1:
                wa = -x_lon[1]/dt - x_lon[2]

            x_lon = np.matmul(A,x_lon) +B*wa

            x_lons[t] = x_lon[0]


            """
            #########################################
            Lateral movement start
            #########################################
            """


            sigma_lats[t] = sigma_limit*np.sqrt(1 - np.exp(-2*alpha*dt*t))*(x_lon[1]/10+1e-4)
            wlat = np.random.normal(0,sigma_lats[t],1)
            x_lat = x_lat*np.exp(-alpha * dt) + wlat
            x_lats[t] = x_lat


        x_lons_global = np.zeros(x_lons.shape)
        x_lats_global = np.zeros(x_lats.shape)
    #     print("====================")
        for j in range(x_lons.shape[0]):
            _s = x_lons[j]
            _d = x_lats[j]

            xy = get_cartesian(_s, _d, route[:,0], route[:,1], lane_maps)

            x_lons_global[j] = xy[0]
            x_lats_global[j] = xy[1]


        plt.plot(traj_veh[k,0], traj_veh[k,1], 'kv-', markersize=15, alpha=0.4)
        plt.plot(x_lons_global[0:-1:5], x_lats_global[0:-1:5], 'rv-', markersize=15, alpha=0.4)


        plt.xlim(clip_val[0])
        plt.ylim(clip_val[1])


        plt.pause(rate)
        plt.cla()

    plt.show()


def main():

    file_name = 'tu'
    rate  = 0.1

    try:
        file_name = sys.argv[1] # sample file idx
    except:
        pass

    try:
        rate = float(sys.argv[2]) # plot speed
    except:
        pass

    file = './sample/'+file_name+'.pickle'

    print('sample file : ', file)

    with open(file, 'rb') as f:
        data = pickle.load(f)

    simulation(data, file_name, rate)




if __name__ == '__main__':
    main()
