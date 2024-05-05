import pygame
import time

def main():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        # No joysticks!
        print("Error, I didn't find any joysticks.")
        return
    else:
        # Use the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

    try:
        while True:
            pygame.event.pump()  # Process event queue
            for i in range(joystick.get_numaxes()):
                axis = joystick.get_axis(i)
                print(f"Axis {i} value: {axis:.2f}")
            for i in range(joystick.get_numbuttons()):
                button = joystick.get_button(i)
                print(f"Button {i} value: {button}")
            time.sleep(0.1)  # Update every 0.1 seconds to avoid flooding the console
    except KeyboardInterrupt:
        print("Program exited.")
        pygame.quit()

if __name__ == "__main__":
    main()
