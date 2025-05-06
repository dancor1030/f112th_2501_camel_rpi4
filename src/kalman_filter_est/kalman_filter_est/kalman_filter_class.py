import numpy as np
from numpy.linalg import inv
import threading
import time


class KalmanFilter():
    def __init__(self,
                 Fmatrix : np.ndarray,
                 Gmatrix : np.ndarray,
                 Pmatrix : np.ndarray,
                 Hmatrix : np.ndarray,
                 Rmatrix : np.ndarray,
                 Qmatrix : np.ndarray,
                 x0 : np.ndarray,
                 dt : float,):
        self.F = Fmatrix
        self.G = Gmatrix
        self.P = Pmatrix
        self.H = Hmatrix
        self.R = Rmatrix
        self.Q = Qmatrix
        self.dt = dt

        self.x = x0
        self.kn = 0. #! <- would be better to prealloc.

        self.u_t = 0.
        self.zn = 0.

        self.measurement_flag = 0

    def update_measurement(self, zn : np.ndarray):
        self.zn = zn
        self.measurement_flag = 1

    def update_measurement(self, u_t : np.ndarray):
        self.u_t = u_t

    def __kf_update(self):
        while(1):
            self.__time_update(u_t)
            if self.measurement_flag:
                self.__measurement_update()
            time.sleep(self.dt)

    def __time_update(self, u_t : np.ndarray):
        self.P = self.F*self.P*self.F.transpose() + self.Q
        self.x = self.F*self.x + u_t

    def __measurement_update(self, zn : np.ndarray):
        self.kn = self.P*self.H.transpose()*inv(self.H*self.P*self.H.transpose() + self.R)
        self.x = self.x + self.kn*(zn - self.H*self.x)
        self.P = (np.ones_like(self.P) - self.kn*self.H)*self.P*(np.ones_like(self.P) - self.kn*self.H).transpose() + self.kn*self.R*self.kn.transpose()
        self.measurement_flag = 0

    def __start_kf(self):
        kf_thread = threading.Thread(target=Self.__kf_update, args=())
        kf_thread.start()
