import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from params import *

def equations(vars, l1, l2, l3, h, r):
    """
    Define the system of equations to solve for a 3-segment robotic arm
    vars = [theta1, theta2, theta3]
    
    l1, l2, l3 are the lengths of the three arm segments
    h is the height of the base
    (r, 0) is the target point for the end-effector
    """
    theta1, theta2, theta3 = vars
    
    # Calculate end-effector position
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    y = h + l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
    
    # End-effector should reach target point (r, 0)
    eq1 = x - r
    eq2 = y - 0
    
    # Additional constraint - we can choose various constraints
    # Here we'll use a constraint that the end-effector stays horizontal (suction cup facing down)
    # This means theta1 + theta2 + theta3 = -π/2 (pointing downward)
    eq3 = theta1 + theta2 + theta3 + np.pi/2
    
    return [eq1, eq2, eq3]

def solve_arm_kinematics(l1, l2, l3, h, r, initial_guess=None):
    """
    Solve for joint angles given target point
    
    Parameters:
    -----------
    l1, l2, l3 : float
        Lengths of the three arm segments
    h : float
        Height of the base
    r : float
        Target x-coordinate (target point is (r, 0))
    initial_guess : list, optional
        Initial guess for [theta1, theta2, theta3] in radians
        
    Returns:
    --------
    theta1, theta2, theta3 : float
        Joint angles in radians
    """
    if initial_guess is None:
        # Provide a reasonable initial guess
        initial_guess = [0.0, 0.0, -np.pi/2]
    
    # Solve the system of equations
    solution, info, ier, mesg = fsolve(equations, initial_guess, args=(l1, l2, l3, h, r), full_output=True)
    
    if ier != 1:
        print(f"Warning: Solver did not converge. Message: {mesg}")
    
    theta1, theta2, theta3 = solution
    
    # Normalize angles to be in the range [-π, π]
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
    theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))
    theta3 = np.arctan2(np.sin(theta3), np.cos(theta3))
    
    return theta1, theta2, theta3

def visualize_robotic_arm(l1, l2, l3, theta1, theta2, theta3, h, r):
    """
    Visualize the robotic arm configuration
    
    Parameters:
    -----------
    l1, l2, l3 : float
        Lengths of the three arm segments
    theta1, theta2, theta3 : float
        Joint angles in radians
    h : float
        Height of the base
    r : float
        Target x-coordinate
    """
    # Calculate joint positions
    x0, y0 = 0, h  # Base at height h
    
    x1 = l1 * np.cos(theta1)
    y1 = h + l1 * np.sin(theta1)
    
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    
    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    
    # Plot
    plt.figure(figsize=(10, 6))
    
    # Draw arm segments
    plt.plot([x0, x1], [y0, y1], 'b-', linewidth=3, label='Link 1 (l₁)')
    plt.plot([x1, x2], [y1, y2], 'g-', linewidth=3, label='Link 2 (l₂)')
    plt.plot([x2, x3], [y2, y3], 'r-', linewidth=3, label='Link 3 (l₃)')
    
    # Draw target
    plt.plot(r, 0, 'rx', markersize=12, label='Target')
    
    # Mark joints
    plt.plot(x0, y0, 'ko', markersize=10)
    plt.plot(x1, y1, 'ko', markersize=8)
    plt.plot(x2, y2, 'ko', markersize=8)
    
    # Mark end-effector
    plt.plot(x3, y3, 'mo', markersize=10, label='End-effector')
    
    # Draw suction cup (a small circle at the end-effector)
    suction_circle = plt.Circle((x3, y3), 0.2, color='m', fill=False, linestyle='--')
    plt.gca().add_patch(suction_circle)
    
    # Add labels
    plt.text(x0-0.1, y0-0.4, 'Base', fontsize=12)
    plt.text(x1+0.1, y1+0.1, 'Joint 1', fontsize=10)
    plt.text(x2+0.1, y2+0.1, 'Joint 2', fontsize=10)
    plt.text(x3+0.1, y3+0.1, 'Suction', fontsize=10)
    
    # Add angle labels
    plt.text(x0+0.5*np.cos(theta1/2), y0+0.5*np.sin(theta1/2), f'θ₁={np.degrees(theta1):.1f}°', fontsize=10)
    plt.text(x1+0.5*np.cos((theta1+theta2)/2), y1+0.5*np.sin((theta1+theta2)/2), f'θ₂={np.degrees(theta2):.1f}°', fontsize=10)
    plt.text(x2+0.5*np.cos((theta1+theta2+theta3)/2), y2+0.5*np.sin((theta1+theta2+theta3)/2), f'θ₃={np.degrees(theta3):.1f}°', fontsize=10)
    
    # Add ground line
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # Add height label
    plt.text(-0.5, h/2, f'h={h}', fontsize=10)
    
    # Draw coordinate at end-effector
    plt.text(x3+0.2, y3-0.4, f'({x3:.2f}, {y3:.2f})', fontsize=9)
    
    # Draw coordinate at target
    plt.text(r+0.2, 0-0.4, f'({r:.2f}, 0.00)', fontsize=9)
    
    plt.grid(True)
    plt.axis('equal')
    plt.legend(loc='upper left')
    plt.title('Three-Segment Robotic Arm with Suction End-Effector')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    
    # Set view limits
    max_length = max(l1, l2, l3) * 2
    max_range = max(l1 + l2 + l3, r) * 1.2
    plt.xlim(-max_length * 0.2, max_range)
    plt.ylim(-max_length * 0.5, max_length * 1.2)
    
    plt.show()

def check_reachability(l1, l2, l3, h, r):
    """
    Check if the target point is reachable by the arm
    
    Parameters:
    -----------
    l1, l2, l3 : float
        Lengths of the three arm segments
    h : float
        Height of the base
    r : float
        Target x-coordinate
    
    Returns:
    --------
    bool:
        True if reachable, False otherwise
    """
    # Maximum reach of the arm
    max_reach = l1 + l2 + l3
    
    # Minimum reach (conservative estimate)
    min_reach = max(0, l1 - l2 - l3)
    
    # Distance to target
    target_distance = np.sqrt(r**2 + h**2)
    
    if target_distance > max_reach:
        return False, "Target is beyond maximum reach"
    
    if target_distance < min_reach:
        return False, "Target is too close (within minimum reach)"
    
    # Additional constraint: the arm needs to reach ground level
    if h > l1 + l2 + l3:
        return False, "Arm cannot reach ground level due to base height"
    
    return True, "Target is potentially reachable"

def main():
    # Example values for a robotic arm
    l1 = lengths[1]  # Length of arm segment 1s
    l2 = lengths[2]  # Length of arm segment 2
    l3 = lengths[3]  # Length of arm segment 3 (with suction)
    h = lengths[0]   # Height of the base
    r = 35  # Target x-coordinate (target point is at (r, 0))
    
    print(f"Robotic Arm Parameters:")
    print(f"l1 = {l1} (first segment)")
    print(f"l2 = {l2} (second segment)")
    print(f"l3 = {l3} (third segment with suction)")
    print(f"h = {h} (base height)")
    print(f"Target point: ({r}, 0)")
    
    # Check if target is reachable
    reachable, message = check_reachability(l1, l2, l3, h, r)
    print(f"\nReachability check: {message}")
    
    if not reachable:
        print("The target may not be reachable with the current configuration.")
        print("You can still attempt to solve, but the solver might not converge.")
    
    # Find multiple solutions with different initial guesses
    initial_guesses = [
        [0.0, 0.0, -np.pi/2],  # Arm straight out, end-effector down
        [-np.pi/4, -np.pi/4, 0],  # Arm angled down
        [np.pi/4, -np.pi/2, -np.pi/4],  # Arm angled up
        [-np.pi/2, 0, 0]  # Arm straight down
    ]
    
    print("\nSearching for possible arm configurations...")
    
    solutions = []
    for i, guess in enumerate(initial_guesses):
        try:
            theta1, theta2, theta3 = solve_arm_kinematics(l1, l2, l3, h, r, guess)
            
            # Check if this is a new solution
            is_new = True
            for sol in solutions:
                if (abs(sol[0] - theta1) < 0.01 and 
                    abs(sol[1] - theta2) < 0.01 and 
                    abs(sol[2] - theta3) < 0.01):
                    is_new = False
                    break
            
            if is_new:
                solutions.append((theta1, theta2, theta3))
                print(f"\nSolution {len(solutions)}:")
                print(f"  θ₁ = {theta1:.4f} rad = {np.degrees(theta1):.2f}°")
                print(f"  θ₂ = {theta2:.4f} rad = {np.degrees(theta2):.2f}°")
                print(f"  θ₃ = {theta3:.4f} rad = {np.degrees(theta3):.2f}°")
                
                # Verify the solution
                result = equations([theta1, theta2, theta3], l1, l2, l3, h, r)
                print(f"  Error in x-position: {result[0]:.8f}")
                print(f"  Error in y-position: {result[1]:.8f}")
                print(f"  Error in end-effector orientation: {result[2]:.8f}")
                
                # Calculate end position
                x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
                y = h + l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
                print(f"  End-effector position: ({x:.4f}, {y:.4f})")
        except Exception as e:
            print(f"Failed to find solution with guess {guess}: {e}")
            continue
    
    if solutions:
        # Visualize the first solution
        visualize_robotic_arm(l1, l2, l3, solutions[0][0], solutions[0][1], solutions[0][2], h, r)
    else:
        print("No solutions found. The arm cannot reach the target point with the current configuration.")
    
    # # Interactive tool to try different parameters
    # print("\nWould you like to try different parameters? (yes/no)")
    # response = input().lower()
    
    # if response.startswith('y'):
    #     print("Enter new parameters (press Enter to keep current value):")
        
    #     input_l1 = input(f"l1 (first segment) [{l1}]: ")
    #     if input_l1: l1 = float(input_l1)
        
    #     input_l2 = input(f"l2 (second segment) [{l2}]: ")
    #     if input_l2: l2 = float(input_l2)
        
    #     input_l3 = input(f"l3 (third segment with suction) [{l3}]: ")
    #     if input_l3: l3 = float(input_l3)
        
    #     input_h = input(f"h (base height) [{h}]: ")
    #     if input_h: h = float(input_h)
        
    #     input_r = input(f"r (target x-coordinate) [{r}]: ")
    #     if input_r: r = float(input_r)
        
    #     # Run again with new parameters
    #     main()

if __name__ == "__main__":
    main()