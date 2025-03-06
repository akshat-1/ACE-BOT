import numpy as np
from params import *

def forward_kinematics(thetas):
    theta1, theta2, theta3, theta4, theta5 = thetas
    c1,c2,c3,c4,c5,s1,s2,s3,s4,s5 = np.cos(theta1), np.cos(theta2), np.cos(theta3), np.cos(theta4), np.cos(theta5), np.sin(theta1), np.sin(theta2),np.sin(theta3), np.sin(theta4), np.sin(theta5)
    l0,l1,l2,l3, l4 = lengths[0], lengths[1], lengths[2], lengths[3], lengths[4]

    Tx =  c1*c2*l1 + 0.44807361612917*l1*s1*s2 + l2*(-0.893996663600558*c3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) - 0.400576317866915*s1 + 0.893996663600558*s3*(c1*c2 + 0.44807361612917*s1*s2)) + l3*(0.400576317866915*c3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) - 0.893996663600558*c4*(c3*(c1*c2 + 0.44807361612917*s1*s2) + s3*(-1.0*c1*s2 + 0.44807361612917*c2*s1)) + 0.179487679282337*s1 - 0.400576317866915*s3*(c1*c2 + 0.44807361612917*s1*s2) - 0.893996663600558*s4*(-0.44807361612917*c3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) + 0.799230034528929*s1 + 0.44807361612917*s3*(c1*c2 + 0.44807361612917*s1*s2)))
    Ty = -0.44807361612917*c1*l1*s2 + c2*l1*s1 + l2*(0.400576317866915*c1 - 0.893996663600558*c3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) + 0.893996663600558*s3*(-0.44807361612917*c1*s2 + c2*s1)) + l3*(-0.179487679282337*c1 + 0.400576317866915*c3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) - 0.893996663600558*c4*(c3*(-0.44807361612917*c1*s2 + c2*s1) + s3*(-0.44807361612917*c1*c2 - 1.0*s1*s2)) - 0.400576317866915*s3*(-0.44807361612917*c1*s2 + c2*s1) - 0.893996663600558*s4*(-0.799230034528929*c1 - 0.44807361612917*c3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) + 0.44807361612917*s3*(-0.44807361612917*c1*s2 + c2*s1)))
    
    Tz = l0 + 0.893996663600558*l1*s2 + l2*(-0.799230034528929*c2*c3 + 0.799230034528929*s2*s3 + 0.200769965471071) + l3*(0.358113891690419*c2*c3 - 0.893996663600558*c4*(0.893996663600558*c2*s3 + 0.893996663600558*c3*s2) - 0.358113891690419*s2*s3 - 0.893996663600558*s4*(-0.400576317866915*c2*c3 + 0.400576317866915*s2*s3 - 0.400576317866915) - 0.0899597244387514)
   
    return np.array([Tx, Ty, Tz])

def compute_jacobian(thetas):
    J = np.zeros((3, 5))
    theta1, theta2, theta3, theta4, theta5 = thetas
    c1,c2,c3,c4,s1,s2,s3,s4 = np.cos(theta1), np.cos(theta2), np.cos(theta3), np.cos(theta4), np.sin(theta1), np.sin(theta2),np.sin(theta3), np.sin(theta4)
    l1,l2,l3 = lengths[1], lengths[2], lengths[3], lengths[4]


    # Partial derivatives for Tx
    J[0, 0] = -c1*c2*l1 + 0.44807361612917*l1*c1*s2 + l2*(0.893996663600558*c3*(s1*s2 + 0.44807361612917*c2*c1) - 0.400576317866915*c1 + 0.893996663600558*s3*(-s1*c2 + 0.44807361612917*c1*s2)) + l3*(-0.400576317866915*c3*(s1*s2 + 0.44807361612917*c2*c1) - 0.893996663600558*c4*(c3*(-s1*c2 + 0.44807361612917*c1*s2) + s3*(s1*s2 + 0.44807361612917*c2*c1)) + 0.179487679282337*c1 + 0.400576317866915*s3*(-s1*c2 + 0.44807361612917*c1*s2) - 0.893996663600558*s4*(0.44807361612917*c3*(s1*s2 + 0.44807361612917*c2*c1) + 0.799230034528929*c1 - 0.44807361612917*s3*(-s1*c2 + 0.44807361612917*c1*s2)))
    J[0, 1] = -s2*c1*l1 + 0.44807361612917*l1*s1*c2 + l2*(-0.893996663600558*c3*(-1.0*c1*c2 - 0.44807361612917*s2*s1) + 0.893996663600558*s3*(-s2*c1 + 0.44807361612917*c2*s1)) + l3*(0.400576317866915*c3*(-1.0*c1*c2 - 0.44807361612917*s2*s1) - 0.893996663600558*c4*(c3*(-s2*c1 + 0.44807361612917*c2*s1) - s3*(-1.0*c1*c2 - 0.44807361612917*s2*s1)) + 0.400576317866915*s3*(-s2*c1 + 0.44807361612917*c2*s1) - 0.893996663600558*s4*(-0.44807361612917*c3*(-1.0*c1*c2 - 0.44807361612917*s2*s1) + 0.44807361612917*s3*(-s2*c1 + 0.44807361612917*c2*s1)))
    J[0, 2] = l2*(-0.893996663600558*s3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) + 0.893996663600558*c3*(c1*c2 + 0.44807361612917*s1*s2)) + l3*(0.400576317866915*s3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) - 0.893996663600558*c4*(-s3*(c1*c2 + 0.44807361612917*s1*s2) + c3*(-1.0*c1*s2 + 0.44807361612917*c2*s1)) - 0.400576317866915*c3*(c1*c2 + 0.44807361612917*s1*s2) - 0.893996663600558*s4*(-0.44807361612917*s3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) + 0.44807361612917*c3*(c1*c2 + 0.44807361612917*s1*s2)))
    J[0, 3] = l3*(0.893996663600558*s4*(c3*(c1*c2 + 0.44807361612917*s1*s2) + s3*(-1.0*c1*s2 + 0.44807361612917*c2*s1)) - 0.893996663600558*c4*(-0.44807361612917*c3*(-1.0*c1*s2 + 0.44807361612917*c2*s1) + 0.799230034528929*s1 + 0.44807361612917*s3*(c1*c2 + 0.44807361612917*s1*s2)))
    J[0, 4] = 0

    # Partial derivatives for Ty
    J[1, 0] = 0.44807361612917*s1*l1*s2 + c2*l1*c1 + l2*(-0.400576317866915*s1 - 0.893996663600558*c3*(0.44807361612917*s1*c2 - 1.0*c1*s2) + 0.893996663600558*s3*(0.44807361612917*s1*s2 + c2*c1)) + l3*(0.179487679282337*s1 - 0.400576317866915*c3*(0.44807361612917*s1*c2 - 1.0*c1*s2) - 0.893996663600558*c4*(c3*(0.44807361612917*s1*s2 + c2*c1) + s3*(0.44807361612917*s1*c2 - 1.0*c1*s2)) + 0.400576317866915*s3*(0.44807361612917*s1*s2 + c2*c1) - 0.893996663600558*s4*(0.799230034528929*s1 - 0.44807361612917*c3*(0.44807361612917*s1*c2 - 1.0*c1*s2) + 0.44807361612917*s3*(0.44807361612917*s1*s2 + c2*c1)))

    J[1, 1] = -0.44807361612917*c1*l1*c2 - s2*l1*s1 + l2*(-0.893996663600558*c3*(0.44807361612917*c1*s2 - 1.0*s1*c2) + 0.893996663600558*s3*(-0.44807361612917*c1*c2 - s2*s1)) + l3*(0.400576317866915*c3*(0.44807361612917*c1*s2 - 1.0*s1*c2) - 0.893996663600558*c4*(c3*(-0.44807361612917*c1*c2 - s2*s1) + s3*(0.44807361612917*c1*s2 - 1.0*s1*c2)) - 0.400576317866915*s3*(-0.44807361612917*c1*c2 - s2*s1) - 0.893996663600558*s4*(-0.44807361612917*c3*(0.44807361612917*c1*s2 - 1.0*s1*c2) + 0.44807361612917*s3*(-0.44807361612917*c1*c2 - s2*s1)))


    J[1, 2] = l2*(0.893996663600558*s3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) + 0.893996663600558*c3*(-0.44807361612917*c1*s2 + c2*s1)) + l3*(-0.400576317866915*s3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) - 0.893996663600558*c4*(-s3*(-0.44807361612917*c1*s2 + c2*s1) + c3*(-0.44807361612917*c1*c2 - 1.0*s1*s2)) - 0.400576317866915*c3*(-0.44807361612917*c1*s2 + c2*s1) - 0.893996663600558*s4*(0.44807361612917*s3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) + 0.44807361612917*c3*(-0.44807361612917*c1*s2 + c2*s1)))
    
    J[1, 3] = l3*(0.893996663600558*s4*(c3*(-0.44807361612917*c1*s2 + c2*s1) + s3*(-0.44807361612917*c1*c2 - 1.0*s1*s2)) + 0.893996663600558*c4*(-0.799230034528929*c1 - 0.44807361612917*c3*(-0.44807361612917*c1*c2 - 1.0*s1*s2) + 0.44807361612917*s3*(-0.44807361612917*c1*s2 + c2*s1)))
    J[1, 4] = 0
    
    # Partial derivatives for Tz
    J[2, 0] = 0
    J[2, 1] = 0.893996663600558*l1*c2 + l2*(-0.799230034528929*(-s2*c3) + 0.799230034528929*c2*s3) + l3*(0.358113891690419*(-s2*c3) - 0.893996663600558*c4*(0.893996663600558*(-s2*s3) + 0.893996663600558*(-c2*c3)) - 0.358113891690419*c2*s3 - 0.893996663600558*s4*(-0.400576317866915*(-s2*c3) - 0.400576317866915*c2*s3))
    J[2, 2] = l2*(-0.799230034528929*(-c2*s3) - 0.799230034528929*s2*c3) + l3*(0.358113891690419*(-c2*s3) - 0.893996663600558*c4*(0.893996663600558*(c2*c3) + 0.893996663600558*(-s2*s3)) - 0.358113891690419*(-s2*c3) - 0.893996663600558*s4*(-0.400576317866915*(-c2*s3) - 0.400576317866915*(-s2*c3)))

    J[2, 3] =l3*(-0.893996663600558*s4*(0.893996663600558*c2*s3 + 0.893996663600558*c3*s2) - 0.893996663600558*c4*(-0.400576317866915*c2*c3 + 0.400576317866915*s2*s3 - 0.400576317866915))

    J[2, 4] = 0
 

    return J

def inv_kin(desired_end_effector_position, initial_guess=None):
    if initial_guess is None:
        initial_guess = np.array([0.1, 0.1, 0.1, 0, 0])
    
    thetas = initial_guess
    max_iterations = 1000
    threshold = 1e-4
    damping_factor = 0.01

    for i in range(max_iterations):
        current_position = forward_kinematics(thetas)
        error = desired_end_effector_position - current_position

        if np.linalg.norm(error) < threshold:
            
            return thetas

        J = compute_jacobian(thetas)
        J_damped = J.T @ np.linalg.inv(J @ J.T + damping_factor**2 * np.eye(3))
        delta_thetas = J_damped @ error

        thetas += delta_thetas

     
        thetas = np.clip(thetas, [-np.pi, -np.pi/2, -np.pi, -np.pi, -np.pi, -np.pi], [np.pi, np.pi/2, np.pi, np.pi, np.pi, np.pi])

   
