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

##################################################################
#              TODO: Connect to joystick                         #
##################################################################

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
