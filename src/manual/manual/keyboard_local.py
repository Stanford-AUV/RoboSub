### RUN LOCALLY ###

import nats
import asyncio
from manual.utils.keyboard import KeyboardState
import keyboard

SEND_INTERVAL_MS = 50  # Send every 50 ms


def isKeyPressed(a):
    return keyboard.is_pressed(a)


keysToState = {
    "a": KeyboardState.moveLeft,
    "d": KeyboardState.moveRight,
    "w": KeyboardState.moveForward,
    "s": KeyboardState.moveBackward,
    "r": KeyboardState.moveUp,
    "f": KeyboardState.moveDown,
    "q": KeyboardState.turnCCW,
    "e": KeyboardState.turnCW,
    "t": KeyboardState.increaseVel,
    "g": KeyboardState.decreaseVel,
    "y": KeyboardState.increaseLinVel,
    "h": KeyboardState.decreaseLinVel,
    "u": KeyboardState.increaseAngVel,
    "j": KeyboardState.decreaseAngVel,
}


async def main():
    # Connect to NATS
    nc = await nats.connect("nats://localhost:4222")
    print("✅ Connected to NATS server")

    try:
        while True:
            state = KeyboardState()
            for key in keysToState:
                state.keysToState[key] = 0
                if isKeyPressed(key):
                    state.keysToState[key] = 1
            try:
                await nc.publish("keyboard", state.to_json().encode("utf-8"))
                print(f"Sent:{state}")
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
