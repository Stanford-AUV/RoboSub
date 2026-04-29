### RUN LOCALLY ###

import pygame
import asyncio
import nats
from manual.utils.joystick import JoystickState

DEADZONE = 0.1  # Ignore tiny joystick noise
SEND_INTERVAL_MS = 50  # Send every 50 ms

# --- Init Pygame & Joystick ---
pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(count)

joystick = pygame.joystick.Joystick(0)


# CONNECT TO JOYSTICK
print(joystick.get_name())
print(joystick.get_axis(0))


async def main():
    # Connect to NATS
    nc = await nats.connect("nats://localhost:4222")
    print("✅ Connected to NATS server")

    try:
        while True:
            pygame.event.pump()
            state = JoystickState()

            ##################################################################
            #           TODO: Calculate, send joystick state                 #
            ##################################################################
            num_axes = joystick.get_numaxes()
            axes = []
            # retrieve axes data, round
            for i in range(num_axes):
                axes.append(joystick.get_axis(i))

            # retrieve button states
            num_buttons = joystick.get_numbuttons()
            buttons = []

            for i in range(num_buttons):
                print(joystick.get_button(i))
                buttons.append(joystick.get_button(i))

            state = JoystickState(axes=axes, buttons=buttons)

            try:
                await nc.publish("joystick", state.to_json().encode("utf-8"))
                print(f"Sent: {state}")
            except Exception as e:
                print("Failed to send:", e)

            await asyncio.sleep(SEND_INTERVAL_MS / 1000.0)

    finally:
        print("Closing NATS connection.")
        await nc.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        pygame.quit()
