import serial
import time

# Replace 'COM3' with your Arduino's serial port (e.g., '/dev/ttyACM0' for Linux or '/dev/tty.usbmodem14201' for macOS)
serial_port = 'COM3'
baud_rate = 115200

def send_speed(speed):
    formatted_speed = "<[{}]>".format(speed)
    print("Sending speed:", formatted_speed)
    ser.write(formatted_speed.encode())

if __name__ == "__main__":
    ser = serial.Serial(serial_port, baud_rate)
    time.sleep(2)  # wait for the serial connection to initialize

    try:
        while True:
            speed_input = input("Enter speed value: ")
            try:
                speed = float(speed_input)
                send_speed(speed)
            except ValueError:
                print("Please enter a valid number.")
    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        ser.close()
