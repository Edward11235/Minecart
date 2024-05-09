import pygame
import serial
import time

# Initialize Pygame for joystick reading
pygame.init()
pygame.joystick.init()

# Ensure you have at least one joystick connected
if pygame.joystick.get_count() < 1:
    print("Please connect a joystick.")
    pygame.quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Replace 'COM_PORT' with your Bluetooth COM port
serial_port_1 = '/dev/ttyUSB0'
serial_port_2 = '/dev/ttyUSB1'
baud_rate = 115200

# Setup serial connection (adjust COM port as necessary)
ser_1 = serial.Serial(serial_port_1, baud_rate)
ser_2 = serial.Serial(serial_port_2, baud_rate)

time.sleep(2)  # Wait for the serial connection to initialize

def send_speed(speed, turn):
    """Send speed and turn values to the rover's motors.

    Args:
        speed (float): Speed value from the joystick's Y-axis (-100 to 100).
        turn (float): Turn value from the joystick's X-axis (-100 to 100).
    """
    # Calculate control signals for left and right wheels
    control_signal_left = speed + turn
    control_signal_right = speed - turn

    # Format the messages for left and right wheels
    message_left = f"<[{-control_signal_left}]>"
    message_right = f"<[{control_signal_right}]>"
    
    # Send the messages to the respective serial ports
    ser_1.write(message_left.encode())  # Send to left wheel motor
    time.sleep(0.01)  # Short delay to ensure serial communication stability
    ser_2.write(message_right.encode())  # Send to right wheel motor

try:
    while True:
        pygame.event.pump()  # Update Pygame events

        # Get the vertical position of the left joystick (values range from -1 to 1)
        joystick_y = joystick.get_axis(1)  # Axis 1 is typically the vertical axis for the left joystick
        # Get the horizontal position of the left joystick
        joystick_x = joystick.get_axis(0)  # Axis 0 is typically the horizontal axis for the left joystick
        # print(joystick_y, joystick_x)

        # Calculate speed based on joystick position
        if -0.1 < joystick_y < 0.1:  # Pushed to the top
           send_speed(0, 0)
        else:
            speed_for_motor = int(50*joystick_y)
            if -0.1 < joystick_x < 0.1:
                send_speed(speed_for_motor, 0)
            else:
                turn_for_motor = int(20*joystick_x)
                send_speed(speed_for_motor, turn_for_motor)
            
        time.sleep(0.1)  # Add a small delay to limit the command sending rate

except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser_1.close()
    ser_2.close()
    pygame.quit()
