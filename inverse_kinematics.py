from numpy import cos ,sin
import numpy as np
from params import *

def forward_kinematics(thetas):
    t1, t2, t3, t4, t5 = thetas
    # c1,c2,c3,c4,c5,s1,s2,s3,s4,s5 = np.cos(theta1), np.cos(theta2), np.cos(theta3), np.cos(theta4), np.cos(theta5), np.sin(theta1), np.sin(theta2),np.sin(theta3), np.sin(theta4), np.sin(theta5)
    l0,l1,l2,l3,l4,l5 = lengths[0], lengths[1], lengths[2], lengths[3], lengths[4], lengths[5]

    Tx =  0.44807361612917*l2*sin(t1)*sin(t2) + l2*cos(t1)*cos(t2) + l4*(0.893996663600558*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) - 0.400576317866915*sin(t1)) - l5*(-0.893996663600558*(-(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + (0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3))*sin(t4) - 0.400576317866915*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) + 0.893996663600558*(0.44807361612917*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) + 0.799230034528929*sin(t1))*cos(t4) + 0.179487679282337*sin(t1)) + 0.893996663600558*(-l1 + l3)*sin(t1)
    
    Ty = l2*sin(t1)*cos(t2) - 0.44807361612917*l2*sin(t2)*cos(t1) + l4*(0.893996663600558*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3) + 0.400576317866915*cos(t1)) - l5*(-0.893996663600558*((-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - (sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3))*sin(t4) - 0.400576317866915*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3) + 0.893996663600558*(0.44807361612917*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3) - 0.799230034528929*cos(t1))*cos(t4) - 0.179487679282337*cos(t1)) - 0.893996663600558*(-l1 + l3)*cos(t1) 
    Tz = l0 + 0.44807361612917*l1 + 0.893996663600558*l2*sin(t2) - 0.44807361612917*l3 + l4*(0.799230034528929*sin(t2)*sin(t3) + 0.799230034528929*sin(t3)*cos(t2) + 0.200769965471071) - l5*(-0.893996663600558*(-0.893996663600558*sin(t2)*sin(t3) + 0.893996663600558*sin(t3)*cos(t2))*sin(t4) + 0.893996663600558*(0.400576317866915*sin(t2)*sin(t3) + 0.400576317866915*sin(t3)*cos(t2) - 0.400576317866915)*cos(t4) - 0.358113891690419*sin(t2)*sin(t3) - 0.358113891690419*sin(t3)*cos(t2) - 0.0899597244387514)
    return np.array([Tx, Ty, Tz])

def compute_jacobian(thetas):
    J = np.zeros((3, 5))
    theta1, theta2, theta3, theta4, theta5 = thetas
    t1,t2,t3,t4,t5 = theta1,theta2, theta3, theta4, theta5
    l1,l2,l3,l4,l5 = lengths[1], lengths[2], lengths[3], lengths[4], lengths[5]


    # Partial derivatives for Tx
    J[0, 0] = -l2*sin(t1)*cos(t2) + 0.44807361612917*l2*sin(t2)*cos(t1) + l4*(0.893996663600558*(1.0*sin(t1)*sin(t2) + 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(-sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3) - 0.400576317866915*cos(t1)) - l5*(-0.893996663600558*((1.0*sin(t1)*sin(t2) + 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - (-sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3))*sin(t4) - 0.400576317866915*(1.0*sin(t1)*sin(t2) + 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(-sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3) + 0.893996663600558*(0.44807361612917*(1.0*sin(t1)*sin(t2) + 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(-sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3) + 0.799230034528929*cos(t1))*cos(t4) + 0.179487679282337*cos(t1)) + 0.893996663600558*(-l1 + l3)*cos(t1)
    J[0, 1] =  0.44807361612917*l2*sin(t1)*cos(t2) - l2*sin(t2)*cos(t1) + l4*(0.893996663600558*(-0.44807361612917*sin(t1)*sin(t2) - 1.0*cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(0.44807361612917*sin(t1)*cos(t2) - sin(t2)*cos(t1))*sin(t3)) - l5*(0.893996663600558*(0.44807361612917*(-0.44807361612917*sin(t1)*sin(t2) - 1.0*cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(0.44807361612917*sin(t1)*cos(t2) - sin(t2)*cos(t1))*sin(t3))*cos(t4) - 0.893996663600558*((-0.44807361612917*sin(t1)*sin(t2) - 1.0*cos(t1)*cos(t2))*sin(t3) - (0.44807361612917*sin(t1)*cos(t2) - sin(t2)*cos(t1))*sin(t3))*sin(t4) - 0.400576317866915*(-0.44807361612917*sin(t1)*sin(t2) - 1.0*cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(0.44807361612917*sin(t1)*cos(t2) - sin(t2)*cos(t1))*sin(t3))
    J[0, 2] = l4*(0.893996663600558*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*cos(t3) + 0.893996663600558*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*cos(t3)) - l5*(-0.893996663600558*(-(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*cos(t3) + (0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*cos(t3))*sin(t4) + 0.893996663600558*(0.44807361612917*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*cos(t3) + 0.44807361612917*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*cos(t3))*cos(t4) - 0.400576317866915*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*cos(t3) - 0.400576317866915*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*cos(t3))
    J[0, 3] = -l5*(-0.893996663600558*(-(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + (0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3))*cos(t4) - 0.893996663600558*(0.44807361612917*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) + 0.799230034528929*sin(t1))*sin(t4))
    J[0, 4] = 0

    # Partial derivatives for Ty
    J[1, 0] =0.44807361612917*l2*sin(t1)*sin(t2) + l2*cos(t1)*cos(t2) + l4*(0.893996663600558*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) - 0.400576317866915*sin(t1)) - l5*(-0.893996663600558*(-(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + (0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3))*sin(t4) - 0.400576317866915*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) + 0.893996663600558*(0.44807361612917*(0.44807361612917*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(0.44807361612917*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*sin(t3) + 0.799230034528929*sin(t1))*cos(t4) + 0.179487679282337*sin(t1)) + 0.893996663600558*(-l1 + l3)*sin(t1)
    J[1, 1] = -l2*sin(t1)*sin(t2) - 0.44807361612917*l2*cos(t1)*cos(t2) + l4*(0.893996663600558*(-sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.893996663600558*(-1.0*sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3)) - l5*(-0.893996663600558*(-(-sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + (-1.0*sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3))*sin(t4) + 0.893996663600558*(0.44807361612917*(-sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(-1.0*sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3))*cos(t4) - 0.400576317866915*(-sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - 0.400576317866915*(-1.0*sin(t1)*cos(t2) + 0.44807361612917*sin(t2)*cos(t1))*sin(t3))

    J[1, 2] = l4*(0.893996663600558*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*cos(t3) + 0.893996663600558*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*cos(t3)) - l5*(0.893996663600558*(0.44807361612917*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*cos(t3) + 0.44807361612917*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*cos(t3))*cos(t4) - 0.893996663600558*((-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*cos(t3) - (sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*cos(t3))*sin(t4) - 0.400576317866915*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*cos(t3) - 0.400576317866915*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*cos(t3)) 
    J[1, 3] = -l5*(-0.893996663600558*((-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) - (sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3))*cos(t4) - 0.893996663600558*(0.44807361612917*(-1.0*sin(t1)*sin(t2) - 0.44807361612917*cos(t1)*cos(t2))*sin(t3) + 0.44807361612917*(sin(t1)*cos(t2) - 0.44807361612917*sin(t2)*cos(t1))*sin(t3) - 0.799230034528929*cos(t1))*sin(t4)) 
    J[1, 4] = 0
    
    # Partial derivatives for Tz
    J[2, 0] = 0
    J[2, 1] = 0.893996663600558*l2*cos(t2) + l4*(-0.799230034528929*sin(t2)*sin(t3) + 0.799230034528929*sin(t3)*cos(t2)) - l5*(-0.893996663600558*(-0.893996663600558*sin(t2)*sin(t3) - 0.893996663600558*sin(t3)*cos(t2))*sin(t4) + 0.893996663600558*(-0.400576317866915*sin(t2)*sin(t3) + 0.400576317866915*sin(t3)*cos(t2))*cos(t4) + 0.358113891690419*sin(t2)*sin(t3) - 0.358113891690419*sin(t3)*cos(t2))
    J[2, 2] = l4*(0.799230034528929*sin(t2)*cos(t3) + 0.799230034528929*cos(t2)*cos(t3)) - l5*(-0.893996663600558*(-0.893996663600558*sin(t2)*cos(t3) + 0.893996663600558*cos(t2)*cos(t3))*sin(t4) + 0.893996663600558*(0.400576317866915*sin(t2)*cos(t3) + 0.400576317866915*cos(t2)*cos(t3))*cos(t4) - 0.358113891690419*sin(t2)*cos(t3) - 0.358113891690419*cos(t2)*cos(t3)) 
    J[2, 3] =-l5*(-0.893996663600558*(-0.893996663600558*sin(t2)*sin(t3) + 0.893996663600558*sin(t3)*cos(t2))*cos(t4) - 0.893996663600558*(0.400576317866915*sin(t2)*sin(t3) + 0.400576317866915*sin(t3)*cos(t2) - 0.400576317866915)*sin(t4))
    J[2, 4] = 0
 

    return J

def inv_kin(desired_end_effector_position, initial_guess=None):
    desired_end_effector_position = np.array(desired_end_effector_position)
    if initial_guess is None:
        initial_guess = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
    
    thetas = initial_guess
    max_iterations = 1100
    threshold = 1e-4
    damping_factor = 0.01


    for i in range(max_iterations):
        current_position = forward_kinematics(thetas)
        error = desired_end_effector_position - current_position
        if np.linalg.norm(error) < threshold:
            
            return np.rad2deg(thetas)

        J = compute_jacobian(thetas)
        J_damped = J.T @ np.linalg.inv(J @ J.T + damping_factor**2 * np.eye(3))
        delta_thetas = J_damped @ error

        thetas += delta_thetas

     
        thetas = np.clip(thetas, [-np.pi, -np.pi/2, -np.pi, -np.pi, -np.pi], [np.pi, np.pi/2, np.pi, np.pi, np.pi])

   
print(inv_kin([15,16,0]))