from time import perf_counter as gettime
from math import isnan

class ControllerPID():
    def __init__(self, kp, ki, kd, Ts=None): # use Ts in case of discrete controller
        # cont. gains
        self.update_gains(kp, ki, kd, Ts)
        self.error = [0.0, 0.0, 0.0] #* ek, ek-1, ek-2
        self.uk = [0.0, 0.0] #* uk, uk-1

    # define setpoint
    def setpoint(self, sp=0):
        self.sp = sp

    ## discrete control law
    def get_discr_u(self, y): # y is the measured variable
        # compute error and control signal
        self.error[0] = self.sp - y
        self.uk[0] = self.q0*self.error[0] + self.q1*self.error[1] + self.q2*self.error[2] + self.uk[1]
        
        if isnan(self.uk[0]):
            self.uk[0] = 0.0

        # update register variables
        self.uk[1] = self.uk[0]
        self.error[1] = self.error[0]
        self.error[2] = self.error[1]

        # send control signal
        return self.uk[0] 
    

    # continuous control law
    def get_cont_u(self, y):
        # TODO: develop this method in case the user wants a continuous controller
        # self.error = self.sp - y
        # self.u = self.kp*self.error + self.ki*self.integral + self.kd*self.dedt
        # return self.u
        pass

    def update_discr_gains(self):
        ti = self.kp/self.ki
        td = self.kd/self.kp
        # compute discr. gains
        self.q0 = self.kp*(1 + self.Ts/(2*ti) + td/self.Ts)
        self.q1 = self.kp*(self.Ts/(2*ti) - 2*td/self.Ts - 1)
        self.q2 = self.kp*(td/self.Ts)
        print(f'updated discr gains: {self.q0} {self.q1} {self.q2}')

    def update_gains(self, kp, ki, kd, Ts=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.Ts = Ts

        if isinstance(Ts, (int, float)):
            self.update_discr_gains()
        else:
            self.q0 = 0.0
            self.q1 = 0.0
            self.q2 = 0.0           

    ## saturate control law
    def satur(self, minu, maxu, u):
        if u < minu: return minu
        elif u > maxu: return maxu
        else: return u



#? SHOULD WE INCLUDE THIS CLASS ? ------------------------------------------------------------------------ 
class Morodico(): # MObile RObotics DIfferential drive COntrol algorithm
    def __init__(self, Klin=1.0, Kang=1.0, Ts=None): #ctype = 'linear' or 'angular'
        self.kp

    def linvel(self, angerror):
        linvel = self.Klin/(self.Kang * angerror + 1)
        return linvel
    
    def angvel(self):
        pass
#? ------------------------------------------------------------------------------------------------------

