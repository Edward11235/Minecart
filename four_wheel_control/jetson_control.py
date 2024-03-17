from pynput import keyboard
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust as necessary
# time.sleep(2)  # Allow time for the serial connection to establish

# Define initial speeds for each wheel (stopped)
front_left = 0
front_right = 0
back_left = 0
back_right = 0

revert_front_left = -1
revert_front_right = 1
revert_back_left = -1
revert_back_right = 1

def on_press(key):
    global front_left, front_right, back_left, back_right
    try:
        if key == keyboard.Key.up:
            # All wheels forward
            front_left = front_right = back_left = back_right = 255
        elif key == keyboard.Key.down:
            # All wheels backward
            front_left = front_right = back_left = back_right = -255
        elif key == keyboard.Key.left:
            # Turn left
            front_left = back_left = 100
            front_right = back_right = 255
        elif key == keyboard.Key.right:
            # Turn right
            # front_left = back_left = 255
            # front_right = back_right = 100
            front_left = 255
        elif key == keyboard.Key.esc:
            # Exit listener
            return False
        front_left *= revert_front_left
        front_right *= revert_front_right
        back_left *= revert_back_left
        back_right *= revert_back_right
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    # Reset wheel speeds to 0 on key release
    global front_left, front_right, back_left, back_right
    front_left = front_right = back_left = back_right = 0

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    try:
        print("Press arrow keys to control speeds. Press ESC to exit.")
        while listener.running:
            nums = [back_right, front_right, back_left, front_left]
            message = "<[" + "][".join(map(str, nums)) + "]>"
            ser.write(message.encode())  # Send the message
            time.sleep(0.1)  # Short delay to allow for key processing
    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        ser.close()
