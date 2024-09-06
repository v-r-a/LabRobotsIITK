import pygame
import sys

# Initialize Pygame
pygame.init()

# Initialize joystick
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick found.")
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Press buttons on the joystick to find their numbers.")

# Main loop
try:
    while True:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

        # Delay to avoid busy-waiting
        pygame.time.wait(100)

except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()
