import serial
import time
from inputs import get_gamepad

ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust as necessary

# Define initial speeds for each wheel (stopped)
front_left = 0
front_right = 0
back_left = 0
back_right = 0

revert_front_left = -1
revert_front_right = 1
revert_back_left = -1
revert_back_right = 1

def process_gamepad_events():
    global front_left, front_right, back_left, back_right
    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == "Absolute":
                if event.code == "ABS_Y":  # Left joystick vertical
                    value = scale_stick_value(event.state)
                    # Move forward or backward
                    front_left = front_right = back_left = back_right = value
                elif event.code == "ABS_X":  # Left joystick horizontal
                    value = scale_stick_value(event.state)
                    if value < 0:  # Turn left
                        front_left = back_left = 100
                        front_right = back_right = 255
                    elif value > 0:  # Turn right
                        front_left = back_left = 255
                        front_right = back_right = 100
                    else:  # Stop turning
                        front_left = front_right = back_left = back_right = 0
        
        # Apply reversals
        front_left *= revert_front_left
        front_right *= revert_front_right
        back_left *= revert_back_left
        back_right *= revert_back_right
        
        # Send the message
        send_to_rover()

def send_to_rover():
    nums = [back_right, front_right, back_left, front_left]
    message = "<[" + "][".join(map(str, nums)) + "]>"
    ser.write(message.encode())  # Send the message

def scale_stick_value(value):
    """
    Scale gamepad stick value (-32768 to 32767) to motor speed (-255 to 255).
    """
    return value * 255 // 32768

try:
    print("Use the left joystick to control the rover. Exit with CTRL+C.")
    process_gamepad_events()
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()
