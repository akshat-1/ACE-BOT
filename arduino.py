import serial
import time
from inverse_kinematics import inv_kin



# Initialize serial port
serialInst = serial.Serial()
serialInst.port = "COM3"  # Adjust the port if necessary
serialInst.baudrate = 9600  # Ensure this matches the Arduino's baudrate
serialInst.timeout = 1  # Timeout in seconds for reading/writing

# Try to open the serial port and catch any errors
try:
    serialInst.open()
    print(f"Successfully opened {serialInst.portstr}")
except serial.SerialException as e:
    print(f"Error: Could not open serial port {serialInst.portstr} -> {e}")
    exit() 




    command =""
    
    try:
        solutions = inv_kin(desired_end_effector_position=...)
        for i, solution in enumerate(solutions):
            print(f"Solution {i+1} (radians):", solution)
            command+= (str(solution) + ' ')
            plot_robot_arm(link1, link2, solution, f"Solution {i+1}")
        command = command[:-1]
        command += '\n'


    except ValueError as e:
        print(e)

    serialInst.write(command.encode('utf-8'))

# Wait a bit to ensure Arduino receives the data
    time.sleep(1)

    # Close the serial connection after sending
    serialInst.close()