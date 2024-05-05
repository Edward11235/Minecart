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
serial_port = 'COM5'
baud_rate = 115200

# Setup serial connection (adjust COM port as necessary)
ser = serial.Serial(serial_port, baud_rate)
time.sleep(2)  # Wait for the serial connection to initialize

def send_speed(speed):
    """Send speed value to motor."""
    formatted_speed = "<[{}]>".format(speed)
    ser.write(formatted_speed.encode())

try:
    while True:
        pygame.event.pump()  # Update Pygame events

        # Get the vertical position of the left joystick (values range from -1 to 1)
        joystick_y = joystick.get_axis(1)  # Axis 1 is typically the vertical axis for the left joystick

        # Calculate speed based on joystick position
        if -0.1 < joystick_y < 0.1:  # Pushed to the top
           send_speed(0)
        else:
            speed_for_motor = 50*joystick_y
            send_speed(speed_for_motor )  # Neutral position, you might want to adjust this behavior

        time.sleep(0.1)  # Add a small delay to limit the command sending rate

except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()
    pygame.quit()
