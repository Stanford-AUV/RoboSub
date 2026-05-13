### RUN LOCALLY ###

import nats
import asyncio
from manual.utils.keyboard import KeyboardState
from pynput import keyboard

SEND_INTERVAL_MS = 50  # Send every 50 ms



def on_press(key, pressed_keys):
    try:
        pressed_keys.add(key.char)  # letter keys (e.g. 'a', 'd', etc.)
    except AttributeError:
        pass  # handle special keys if needed

def on_release(key, pressed_keys):
    try:
        pressed_keys.discard(key.char)
    except AttributeError:
        pass

def isKeyPressed(a: str, pressed_keys) -> bool:
    return a in pressed_keys


keysToState = {
    "a": "moveLeft",
    "d": "moveRight",
    "w": "moveForward",
    "s": "moveBackward",
    "r": "moveUp",
    "f": "moveDown",
    "q": "turnCCW",
    "e": "turnCW",
    "t": "increaseVel",
    "g": "decreaseVel",
    "y": "increaseLinVel",
    "h": "decreaseLinVel",
    "u": "increaseAngVel",
    "j": "decreaseAngVel",
}


async def main():
    # Connect to NATS
    nc = await nats.connect("nats://localhost:4222")
    print("✅ Connected to NATS server")
    # Track key state
    pressed_keys = set()
    # Start the listener in background
    listener = keyboard.Listener(
        on_press=lambda key: on_press(key, pressed_keys),
        on_release=lambda key: on_release(key, pressed_keys)
    )
    listener.start()

    try:
        while True:
            state = KeyboardState()
            for key in keysToState:
                state.setState(keysToState[key], 0)
                if isKeyPressed(key, pressed_keys):
                    state.setState(keysToState[key], 1)
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
