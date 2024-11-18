# import pyfirmata
# import time

# # Configure the Arduino board
# board = pyfirmata.Arduino('COM4')  # Adjust this to your Arduino's port

# # Configure pins
# pul_pin1 = 9 # Connect to PUL+ on DM542T 4/9
# dirp_pin1 = 7  # Connect to DIR+ on DM542T 3/7
# dirm_pin1 = 5  # Connect to DIR- on DM542T 2/5

# # Configure pins
# pul_pin2 = 4  # Connect to PUL+ on DM542T
# dirp_pin2 = 3  # Connect to DIR+ on DM542T
# dirm_pin2 = 2  # Connect to DIR- on DM542T

# ena_pin1 = None # Connect to ENA+ on DM542T (optional, set to None if not used)

# # Set up the pins
# board.digital[pul_pin1].mode = pyfirmata.OUTPUT
# board.digital[dirp_pin1].mode = pyfirmata.OUTPUT
# board.digital[dirm_pin1].mode = pyfirmata.OUTPUT

# if ena_pin1 is not None:
#     board.digital[ena_pin1].mode = pyfirmata.OUTPUT
#     board.digital[ena_pin1].write(0)  # Enable the driver (active LOW)

# # Motor and driver configuration
# steps_per_revolution = 200  # This is typically 200 for a 1.8° stepper motor
# microstep_division = 1  # Set this to your DM542T's microstep setting

# # Calculate total pulses per revolution
# pulses_per_revolution = steps_per_revolution * microstep_division

# def rotate_motor(revolutions, direction, speed_rpm=60):
#     """
#     Rotate the motor a specified number of revolutions
#     :param revolutions: Number of revolutions (can be fractional)
#     :param direction: True for one direction, False for the opposite
#     :param speed_rpm: Speed in rotations per minute
#     """
#     total_pulses = int(abs(revolutions) * pulses_per_revolution)
#     delay = (60 / (speed_rpm * pulses_per_revolution)) / 2

#     # Set direction
#     board.digital[dirp_pin1].write(direction)
#     board.digital[dirm_pin1].write(not direction)


#     # Pulse the motor
#     for _ in range(total_pulses):
#         board.digital[pul_pin1].write(1)
#         time.sleep(delay)
#         board.digital[pul_pin1].write(0)
#         time.sleep(delay)

# # Test rotation in both directions
# try:
#     print("Testing motor rotation...")

#     # Test rotation in one direction
#     print("Rotating in one direction (1/4 turn)...")
#     rotate_motor(1, True, 3600000)
#     time.sleep(1)

#     # Test rotation in the opposite direction
#     print("Rotating in the opposite direction (1/4 turn)...")
#     rotate_motor(2, False, 3600000)

# except KeyboardInterrupt:
#     if ena_pin1 is not None:
#         board.digital[ena_pin1].write(1)  # Disable the driver
#     board.exit()
#     print("\nMotor control interrupted")
# board.exit()
# print("Motor test completed")


import pyfirmata
import time

# Configure the Arduino board
board = pyfirmata.Arduino('COM5')  # Adjust this to your Arduino's port


# Configure pins
ccw_pin1 = 8  
cw_pin1 = 9  
stop_pin1 = 7  

ccw_pin2 = 5  
cw_pin2 = 6
stop_pin2 = 4  
# Set up the pins
board.digital[ccw_pin1].mode = pyfirmata.OUTPUT
board.digital[cw_pin1].mode = pyfirmata.OUTPUT
board.digital[stop_pin1].mode = pyfirmata.OUTPUT
board.digital[ccw_pin2].mode = pyfirmata.OUTPUT
board.digital[cw_pin2].mode = pyfirmata.OUTPUT
board.digital[stop_pin2].mode = pyfirmata.OUTPUT


    
# board.digital[cw_pin1].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[cw_pin1].write(0)  # Enable the driver (active LOW)
# print("cw 1 enabled")

# board.digital[cw_pin2].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[cw_pin2].write(0)  # Enable the driver (active LOW)
# print("cw 2 enabled")
# time.sleep(5)

board.digital[stop_pin1].write(1)  # Enable the driver (active LOW)
time.sleep(0.1)
board.digital[stop_pin1].write(0)  # Enable the driver (active LOW)
print("cw 1 disabled")

board.digital[stop_pin2].write(1)  # Enable the driver (active LOW)
time.sleep(0.1)
board.digital[stop_pin2].write(0)  # Enable the driver (active LOW)
print("cw 2 disabled")


# board.digital[ccw_pin1].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[ccw_pin1].write(0)  # Enable the driver (active LOW)
# print("ccw 1 enabled")

# board.digital[ccw_pin2].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[ccw_pin2].write(0)  # Enable the driver (active LOW)
# print("ccw 2 enabled")

# time.sleep(5)

# board.digital[stop_pin1].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[stop_pin1].write(0)  # Enable the driver (active LOW)
# print("ccw 1 disabled")

# board.digital[stop_pin2].write(1)  # Enable the driver (active LOW)
# time.sleep(0.1)
# board.digital[stop_pin2].write(0)  # Enable the driver (active LOW)
# print("ccw 2 disabled")



board.exit()

