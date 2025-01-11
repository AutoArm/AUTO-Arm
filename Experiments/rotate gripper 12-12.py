import serial
import time
from pyfirmata import Arduino, SERVO, util

# Define COM port and baud rate
COM_PORT = 'COM3'
turn_control_pin = 12
gripper_control_pin = 13
BAUD_RATE = 9600
arduino_board=Arduino(COM_PORT)
arduino_board.digital[turn_control_pin].mode=SERVO
arduino_board.digital[gripper_control_pin].mode=SERVO


# Function to control servo motor
def rotateservo(pin,angle):
    arduino_board.digital[pin].write(angle)
    time.sleep(0.1)

for _ in range(3):
    # open grip
    rotateservo(gripper_control_pin,0)
    time.sleep(2)
   
    rotateservo(turn_control_pin,180)
    time.sleep(2)

    # close grip
    rotateservo(gripper_control_pin,180)
    time.sleep(2)

    rotateservo(turn_control_pin,0)
    time.sleep(2)

   
arduino_board.exit()