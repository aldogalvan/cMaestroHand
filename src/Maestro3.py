import cython
import sympy as sym
import scipy as sp
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from cmath import *
from dataclasses import dataclass
import cython

# This class implements a complete forward modeling of the maestro including
# optimized link lengths
# optimized forward / inverse backlash models
# a physics informed discrepancy learning

class Maestro3:
    
    def __init__(self):
        
        # initialize SEA MCP
        self.SEA_MCP = SEA()
        # initialize SEA PIP
        self.SEA_PIP = SEA()
        # Initialize Kinematic Loop
        self.idx = M3_Finger()
    
        return
        
    def forward_kinematics(self,phi3,psy2):
        phi4,d1 = self.idx.position_KL1(phi3)
        psy1,c2 = self.idx.intermediate_constraints(d1)
        psy3,psy4 = self.idx.position_KL2(psy2, c2)
        theta_b2_c2 = 2*pi - (pi - psy2) - psy4 - (pi - psy2 - psy3)
        PIP = 2*pi - theta_b2_c2 - psy1 - self.psys
        MCP = 2*pi - self.phi1 - phi4
        #print ("psy2:",psy2,"PIP:",PIP)
        return PIP,MCP
    
    def inverse_kinematics(self,MCP,PIP):
        phi3,d1 = self.idx.position_KL1_inverse(MCP)
        psy1,c2 = self.idx.intermediate_constraints(d1)
        psy2,psy4 = self.idx.position_KL2_inverse(PIP, psy1, c2)
        return phi3,psy2
    
    def visualizer(self,phi3_traj,psy2_traj):
        viz = M3_viz(self.idx)
        viz.plot(viz,phi3_traj,psy2_traj)
        return 

##############################################
    
class SEA:
    
    def __init__(self):
        
        self.R_motor = (0.027 + 0.00066)/2
        self.R_joint = 0.025/2
        self.K_comp = 29.25
        
    def params(self,K_comp, R_motor, R_joint):
        
        self.K_comp = K_comp
        self.R_motor = R_motor
        self.R_joint = R_joint
    

    def forward_backlash(self,input_vec,time,motor_angle_actual,joint_angle_actual):
        c = input_vec
        alpha = self.R_motor / self.R_joint
        c0 = [-1,-1]
        results = sp.optimize.minimize(self.forward_backlash_cost,c,args=(time,motor_angle_actual, joint_angle_actual,alpha))
        c_opt = results.x
        return c_opt
          
    @staticmethod 
    def forward_backlash_cost(input_vec,time,motor_angle_actual,joint_angle_actual,alpha):
        c_r = input_vec[0]
        c_l = input_vec[1]
        motor_angle_actual_derivative = np.gradient(motor_angle_actual)/np.gradient(time)
        joint_angle_predicted = np.zeros(np.size(motor_angle_actual))
        joint_angle_predicted[0] = alpha*motor_angle_actual[0]
        tol = 0.01
        
        for t in range(1,np.size(time)):
            if ((motor_angle_actual_derivative[t] > 0) and
                abs(joint_angle_predicted[t-1] - alpha*motor_angle_actual[t-1] - c_r) < tol):
                joint_angle_predicted[t] = alpha*(motor_angle_actual[t] - c_r)
            elif ((motor_angle_actual_derivative[t] < 0) and (abs(joint_angle_predicted[t-1] - alpha*motor_angle_actual[t-1] - c_l)) < tol):
                joint_angle_predicted[t] = alpha*(motor_angle_actual[t] - c_l)
            else:
                joint_angle_predicted[t] = joint_angle_predicted[t-1]
       
        error = abs(joint_angle_predicted - joint_angle_actual)
        weight_mean = 1
        weight_max = 0
        cost = weight_mean*np.mean(error)
        return cost
    
    def inverse_backlash(self,input_vec,time,motor_angle_actual,joint_angle_actual):
        alpha = self.R_motor / self.R_joint
        
        return
    
    @staticmethod
    def inverse_backlash_cost(self,input_vec,time,motor_angle_actual,joint_angle_actual):
        c_r = input_vec[0]
        c_l = input_vec[1]
        rho = input_vec[2]
        alpha = self.R_motor / self.R_joint
        joint_angle_actual_derivative = np.gradient(joint_angle_actual)/np.gradient(time)
        #joint_angle_actual_derivative = lowpass(joint_angle_actual_derivative, 0.01);
        #joint_angle_actual_derivative = smoothdata(joint_angle_actual_derivative,75)
        tol = 0.01; 
        gamma = 1./(1+exp(-rho*joint_angle_actual_derivative))
        joint_angle_predicted = np.zeros(np.size(motor_angle_actual))
        #for t in range(0,np.size(time)):
         #   motor_angle_predicted[t] = joint_angle_actual[t]/alpha + c_r*gamma[t] + c_l*(1-gamma[t])
        #error = abs(motor_angle_predicted - motor_angle_actual)
        weight_mean = 1
        weight_max = 0
        cost = 0 #weight_mean*np.mean(error)
        return cost
        
##############################################
  
class M3_Finger:
    
    def __init__(self):    
        
        # parameters KL1
        self.a1 = 37.75
        self.b1 = 75
        self.c1 = 15
        self.l1 = 45
        self.phi1 = 50*pi/180
        
        # parameters KL2
        self.a2 = 41.50
        self.b2 = sqrt(15**2+12**2)
        self.d2 = 48.00
        self.psys = atan(15/12)
        
        
    def link_lengths_optimize(self):
        
        return
        
    def position_KL1(self,phi3):
        a1 = self.a1
        b1 = self.b1
        c1 = self.c1
        phi1 = self.phi1

        #ALDO
        #phi4 =2*atan((a1 - b1 + ((a1**2 + b1**2 - c1**2 + 2*a1*b1*(2*cos(phi3/2)**2 - 1))/cos(phi3/2)**4)**(1/2)*(cos(phi3)/2 + 1/2) + b1*(cos(phi3) + 1))/(c1 - b1*sin(phi3)))
        phi4 =2*atan((a1 - b1 - ((a1**2 + b1**2 - c1**2 + 2*a1*b1*(2*cos(phi3/2)**2 - 1))/cos(phi3/2)**4)**(1/2)*(cos(phi3)/2 + 1/2) + b1*(cos(phi3) + 1))/(c1 - b1*sin(phi3)))
        #d1 = -((a1**2 + b1**2 - c1**2 + 2*a1*b1*(2*cos(phi3/2)**2 - 1))/cos(phi3/2)**4)**(1/2)*(cos(phi3)/2 + 1/2)
        d1 = ((a1**2 + b1**2 - c1**2 + 2*a1*b1*(2*cos(phi3/2)**2 - 1))/cos(phi3/2)**4)**(1/2)*(cos(phi3)/2 + 1/2)


        return phi4,d1
    
    def intermediate_constraints(self,d1):
        psy1 = atan(self.c1**2 / (self.l1 - d1))
        c2 = sqrt(self.c1**2 + (self.l1 - d1)**2)
        return psy1,c2
    
    def position_KL2(self,psy2,c2):
        
        a2 = self.a2
        b2 = self.b2
        d2 = self.d2
        
        #ALDO
        #psy3 = 2*atan((((cos(psy2) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 + 2*a2**2*d2**2 - 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*a2**2*d2**2*cos(psy2)**2 + 4*a2*d2**3*cos(psy2) + 4*a2**3*d2*cos(psy2) - 4*a2*b2**2*d2*cos(psy2) - 4*a2*c2**2*d2*cos(psy2))/cos(psy2/2)**4)**(1/2))/2 + 2*a2*b2*sin(psy2))/(a2**2 + 2*cos(psy2)*a2*b2 + 2*cos(psy2)*a2*d2 + b2**2 + 2*b2*d2 - c2**2 + d2**2))
        psy3 = -2*atan((((cos(psy2) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 + 2*a2**2*d2**2 - 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*a2**2*d2**2*cos(psy2)**2 + 4*a2*d2**3*cos(psy2) + 4*a2**3*d2*cos(psy2) - 4*a2*b2**2*d2*cos(psy2) - 4*a2*c2**2*d2*cos(psy2))/cos(psy2/2)**4)**(1/2))/2 - 2*a2*b2*sin(psy2))/(a2**2 + 2*cos(psy2)*a2*b2 + 2*cos(psy2)*a2*d2 + b2**2 + 2*b2*d2 - c2**2 + d2**2))
        #psy4 = -2*atan((((cos(psy2) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 + 2*a2**2*d2**2 - 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*a2**2*d2**2*cos(psy2)**2 + 4*a2*d2**3*cos(psy2) + 4*a2**3*d2*cos(psy2) - 4*a2*b2**2*d2*cos(psy2) - 4*a2*c2**2*d2*cos(psy2))/cos(psy2/2)**4)**(1/2))/2 - 2*a2*c2*sin(psy2))/(a2**2 + 2*cos(psy2)*a2*c2 + 2*cos(psy2)*a2*d2 - b2**2 + c2**2 + 2*c2*d2 + d2**2))
        psy4 = 2*atan((((cos(psy2) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 + 2*a2**2*d2**2 - 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*a2**2*d2**2*cos(psy2)**2 + 4*a2*d2**3*cos(psy2) + 4*a2**3*d2*cos(psy2) - 4*a2*b2**2*d2*cos(psy2) - 4*a2*c2**2*d2*cos(psy2))/cos(psy2/2)**4)**(1/2))/2 + 2*a2*c2*sin(psy2))/(a2**2 + 2*cos(psy2)*a2*c2 + 2*cos(psy2)*a2*d2 - b2**2 + c2**2 + 2*c2*d2 + d2**2))
        return psy3,psy4
    
    def position_KL1_inverse(self,MCP):
        a1 = self.a1
        b1 = self.b1
        c1 = self.c1
        phi1 = self.phi1

        #ALDO
        #phi3 =2*atan((b1*tan(MCP/2 + phi1/2)**2 - b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 - b1 + ((b1 - 2*a1*tan(MCP/2 + phi1/2) + 2*c1*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 + b1*tan(MCP/2 + phi1/2)**2 - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4) - 2*a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2)*(b1 + 2*a1*tan(MCP/2 + phi1/2) - 2*c1*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 + b1*tan(MCP/2 + phi1/2)**2 + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4) + 2*a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2))**(1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2)/(2*(a1*tan(MCP/2 + phi1/2) - b1*tan(MCP/2 + phi1/2) - c1*tan(MCP/2 + phi1/2) + c1*tan(MCP/2 + phi1/2 - (3*pi)/4) + a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 + c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2))))
        phi3 =-2*atan((b1 + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 - b1*tan(MCP/2 + phi1/2)**2 + ((b1 - 2*a1*tan(MCP/2 + phi1/2) + 2*c1*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 + b1*tan(MCP/2 + phi1/2)**2 - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4) - 2*a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2)*(b1 + 2*a1*tan(MCP/2 + phi1/2) - 2*c1*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2 + b1*tan(MCP/2 + phi1/2)**2 + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4) + 2*a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 + 2*c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) + b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2))**(1/2) - b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2)**2)/(2*(a1*tan(MCP/2 + phi1/2) - b1*tan(MCP/2 + phi1/2) - c1*tan(MCP/2 + phi1/2) + c1*tan(MCP/2 + phi1/2 - (3*pi)/4) + a1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - b1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2) - c1*tan(MCP/2 + phi1/2 - (3*pi)/4)*tan(MCP/2 + phi1/2)**2 + c1*tan(MCP/2 + phi1/2 - (3*pi)/4)**2*tan(MCP/2 + phi1/2))))
        #d1 =a1*cos(MCP + phi1) - sin(2*MCP + 2*phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) + 2*cos(MCP + phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) - 2*sin(MCP + phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) + 2*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2)
        d1 = sin(2*MCP + 2*phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) + a1*cos(MCP + phi1) - 2*cos(MCP + phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) + 2*sin(MCP + phi1)*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2) - 2*((a1**2 - a1**2*cos(2*MCP + 2*phi1) - 2*b1**2 + 2*c1**2 + 4*a1*c1*sin(MCP + phi1))/(cos(4*MCP + 4*phi1) + 16*sin(2*MCP + 2*phi1) + 4*2**(1/2)*sin(3*MCP + 3*phi1 + pi/4) - 20*2**(1/2)*cos(MCP + phi1 + pi/4) - 17))**(1/2)
        return phi3,d1

    
    def position_KL2_inverse(self,PIP,psy1,c2):
        
        a2 = self.a2
        b2 = self.b2
        d2 = self.d2
        psys = self.psys
        psy2,psy4 = sym.symbols('psy2,psy4')
        psy3 = psy4 - psy1 - 2*psy2 - PIP - psys + 2*pi
        exp1 = sym.Eq(a2*sym.cos(psy2) - b2*sym.cos(psy3) + d2 - c2*sym.cos(psy4),0)
        exp2 = sym.Eq(a2*sym.sin(psy2) - b2*sym.sin(psy3) - c2*sym.sin(psy4),0)
        print(exp1,exp2)
        ret = sym.nsolve([exp1,exp2])
        
        #ALDO
        #psy2 = 2*atan(((cos(C) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 - 2*a2**2*d2**2 + 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*b2*c2**3*cos(C) + 4*b2**3*c2*cos(C) + 4*b2**2*c2**2*cos(C)**2 - 4*a2**2*b2*c2*cos(C) - 4*b2*c2*d2**2*cos(C))/cos(C/2)**4)**(1/2))/(2*(- a2**2 + 2*a2*d2 + b2**2 + 2*cos(C)*b2*c2 + c2**2 - d2**2)))
        #psy2 = -2*atan(((cos(C) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 - 2*a2**2*d2**2 + 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*b2*c2**3*cos(C) + 4*b2**3*c2*cos(C) + 4*b2**2*c2**2*cos(C)**2 - 4*a2**2*b2*c2*cos(C) - 4*b2*c2*d2**2*cos(C))/cos(C/2)**4)**(1/2))/(2*(- a2**2 + 2*a2*d2 + b2**2 + 2*cos(C)*b2*c2 + c2**2 - d2**2)))
        #psy4 = 2*atan((((cos(C) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 - 2*a2**2*d2**2 + 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*b2*c2**3*cos(C) + 4*b2**3*c2*cos(C) + 4*b2**2*c2**2*cos(C)**2 - 4*a2**2*b2*c2*cos(C) - 4*b2*c2*d2**2*cos(C))/cos(C/2)**4)**(1/2))/2 - 2*b2*d2*sin(C))/(- a2**2 + b2**2 + 2*cos(C)*b2*c2 + 2*cos(C)*b2*d2 + c2**2 + 2*c2*d2 + d2**2))
        #psy4 = -2*atan((((cos(C) + 1)*(-(a2**4 + b2**4 + c2**4 + d2**4 - 2*a2**2*b2**2 - 2*a2**2*c2**2 - 2*a2**2*d2**2 + 2*b2**2*c2**2 - 2*b2**2*d2**2 - 2*c2**2*d2**2 + 4*b2*c2**3*cos(C) + 4*b2**3*c2*cos(C) + 4*b2**2*c2**2*cos(C)**2 - 4*a2**2*b2*c2*cos(C) - 4*b2*c2*d2**2*cos(C))/cos(C/2)**4)**(1/2))/2 + 2*b2*d2*sin(C))/(- a2**2 + b2**2 + 2*cos(C)*b2*c2 + 2*cos(C)*b2*d2 + c2**2 + 2*c2*d2 + d2**2))
        
        return psy2,psy4
    
    def torque_KL1(self):

        return
    
    def torque_KL2(self):
        
        return
    
    def define_jacobian(self):
        
        return
    
    def jacobian(self):
        
        return
        
    
##############################################

class M3_viz:
    
    def __init__(self,m3_f : M3_Finger):    
        self.m3_f = m3_f
        return
    
    @staticmethod
    def plot(self,phi3_traj,psy2_traj):
        
        #first assert indices are equal
        assert np.size(phi3_traj)==np.size(psy2_traj)
        
        
        global p1_; global p2_; global p3_; global p4_;
        global p5_; global p6_; global p7_;
        
        p1_,p2_,p3_,p4_,p5_,p6_,p7_ = self.points_viz(phi3_traj, psy2_traj)
        
        global figure 
        figure = plt.figure()
        
        plt.xlabel("X position")
        plt.ylabel("Y position")
        plt.title("Maestro Animation")
        
        global axes
        axes = plt.axes( xlim=(-150, 150), ylim=( -150 , 150) )
        
            
        def bar(p1_,p2_):
            return ((p1_.x,p1_.y),(p2_.x,p2_.y))
        

        bar_p1_p2 = plt.Polygon(bar(p1_[0],p2_[0]),closed = None, ec = 'blue'  , lw = 3)
        bar_p1_p3 = plt.Polygon(bar(p1_[0],p3_[0]),closed = None, ec = 'red'   , lw = 3)
        bar_p3_p4 = plt.Polygon(bar(p3_[0],p4_[0]),closed = None, ec = 'yellow', lw = 3)
        bar_p2_p4 = plt.Polygon(bar(p2_[0],p4_[0]),closed = None, ec = 'green' , lw = 3)
        bar_p2_p5 = plt.Polygon(bar(p2_[0],p5_[0]),closed = None, ec = 'cyan'  , lw = 3)
        bar_p5_p6 = plt.Polygon(bar(p5_[0],p6_[0]),closed = None, ec = 'gray'  , lw = 3)
        bar_p2_p7 = plt.Polygon(bar(p2_[0],p7_[0]),closed = None, ec = 'purple', lw = 3)
        bar_p6_p7 = plt.Polygon(bar(p6_[0],p7_[0]),closed = None, ec = 'pink'  , lw = 3)

        
        def initial_plot():
            axes.add_patch(bar_p1_p2)
            axes.add_patch(bar_p1_p3)
            axes.add_patch(bar_p3_p4)
            axes.add_patch(bar_p2_p4)
            axes.add_patch(bar_p2_p5)
            axes.add_patch(bar_p5_p6)
            axes.add_patch(bar_p2_p7)
            axes.add_patch(bar_p6_p7)
            return bar_p1_p2,bar_p1_p3,bar_p3_p4,bar_p2_p4,bar_p2_p5,bar_p5_p6,bar_p2_p7,bar_p6_p7
        
        
        def animate_index(i):
            bar_p1_p2.set_xy( bar(p1_[i],p2_[i]) )
            bar_p1_p3.set_xy( bar(p1_[i],p3_[i]) )
            bar_p3_p4.set_xy( bar(p3_[i],p4_[i]) )
            bar_p2_p4.set_xy( bar(p2_[i],p4_[i]) )
            bar_p2_p5.set_xy( bar(p2_[i],p5_[i]) )
            bar_p5_p6.set_xy( bar(p5_[i],p6_[i]) )
            bar_p2_p7.set_xy( bar(p2_[i],p7_[i]) )
            bar_p6_p7.set_xy( bar(p6_[i],p7_[i]) )
            return bar_p1_p2,bar_p1_p3,bar_p3_p4,bar_p2_p4,bar_p2_p5,bar_p5_p6,bar_p2_p7,bar_p6_p7
        
        #anim = animation.FuncAnimation( figure, animate_index, init_func = initial_plot,
         #                        frames = np.size(p1_), interval = 40, blit = True )
        
        plt.show()
        initial_plot()
        for i in range(np.size(p1_)):
            plt.pause(0.01)
            animate_index(i)
            if (not plt.get_fignums()):
                break
        
        return 
    
    # get the points
    def points_viz(self,phi3,psy2):
        
        p1_list = []
        p2_list = []
        p3_list = []
        p4_list = []
        p5_list = []
        p6_list = []
        p7_list = []

        traj_max = np.size(phi3)

        for i in range(traj_max):
            phi4 , d1 = self.m3_f.position_KL1(phi3[i])
            psy1 , c2 = self.m3_f.intermediate_constraints(d1)
            psy3 , psy4 = self.m3_f.position_KL2(psy2[i],c2)

            p1_ = self.p1()
            p1_list.append(p1_)
            p2_ = self.p2(p1_,phi3[i])
            p2_list.append(p2_)
            p3_ = self.p3(p1_)
            p3_list.append(p3_)
            p4_ = self.p4(p3_,d1,phi4)
            p4_list.append(p4_)
            off = phi4 
            p5_ = self.p5(p2_,off)
            p5_list.append(p5_)
            p6_ = self.p6(p5_,psy2[i],off)
            p6_list.append(p6_)
            p7_ = self.p7(p2_,c2,psy4,off)
            p7_list.append(p7_)
            
        return p1_list,p2_list,p3_list,p4_list,p5_list,p6_list,p7_list
        
    
    # point 1 (KL1 - input MCP)
    def p1(self):
        p1_ = Point()
        p1_.x = 0
        p1_.y = 0
        return p1_
        
    # point 2 (KL1)
    def p2(self,p1_,phi3):
        p2_ = Point()
        p2_.x = (p1_.x + self.m3_f.b1*cos(phi3))
        p2_.y = (p1_.y + self.m3_f.b1*sin(phi3))
        return p2_
    
    #point 3 (KL1)
    def p3(self,p1_):
        p3_ = Point()
        p3_.x = (p1_.x - self.m3_f.a1).real
        p3_.y = (p1_.y).real
        return p3_
    
    # point 4 (KL2)
    def p4(self,p3_,d1,phi4):
        p4_ = Point()
        p4_.x = (p3_.x + d1*cos(phi4)).real
        p4_.y = (p3_.y + d1*sin(phi4)).real
        return p4_
    
    # point 5 (KL2 - input PIP)
    def p5(self,p2,off):
        p5_ = Point()
        p5_.x = (p2.x + self.m3_f.d2*cos(off))
        p5_.y = (p2.y + self.m3_f.d2*sin(off))
        return p5_
    
    # point 6 (KL2)
    def p6(self,p5,psy2,off):
        p6_ = Point()
        p6_.x = (p5.x + self.m3_f.a2*cos(psy2 + off))
        p6_.y = (p5.y + self.m3_f.a2*sin(psy2 + off))
        return p6_
    
    # point 7 (KL2)
    def p7(self,p2,c2,psy4,off):
        p7_ = Point()
        p7_.x = (p2.x + c2*cos(psy4 + off))
        p7_.y = (p2.y + c2*sin(psy4 + off))
        return p7_

    

##############################################

@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
