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

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected.")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"✅ Using joystick: {joystick.get_name()}")

# --- Helper ---
def get_axis(index):
    if index < joystick.get_numaxes():
        val = joystick.get_axis(index)
        return val if abs(val) > DEADZONE else 0.0
    return 0.0

async def main():
    # Connect to NATS
    nc = await nats.connect("nats://localhost:4222")
    print("✅ Connected to NATS server")

    try:
        while True:
            pygame.event.pump()

            lx = get_axis(0)
            ly = get_axis(5)
            rx = get_axis(3)
            ry = get_axis(4)
            enabled = get_axis(1) < 0 and get_axis(7) < 0

            state = JoystickState(
                lx=lx,
                ly=ly,
                rx=rx,
                ry=ry,
                enabled=enabled
            )

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
