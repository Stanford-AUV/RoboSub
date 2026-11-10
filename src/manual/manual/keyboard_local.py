### RUN LOCALLY ###

import nats
import asyncio
from manual.utils.keyboard import KeyboardState

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


def is_key_pressed(a: str, pressed_keys) -> bool:
    return a in pressed_keys


keys_to_state = {
    "a": "move_left",
    "d": "move_right",
    "w": "move_forward",
    "s": "move_backward",
    "r": "move_up",
    "f": "move_down",
    "q": "turn_ccw",  # turn_cw
    "e": "turn_cw",  # turn_ccw
    "t": "increase_vel",
    "g": "decrease_vel",
    "y": "increase_lin_vel",
    "h": "decrease_lin_vel",
    "u": "increase_ang_vel",
    "j": "decrease_ang_vel",
}


async def main():
    from pynput import keyboard

    # Connect to NATS
    nc = await nats.connect("nats://localhost:4222")
    print("Connected to NATS server")
    # Track key state
    pressed_keys = set()
    # Start the listener in background
    listener = keyboard.Listener(
        on_press=lambda key: on_press(key, pressed_keys),
        on_release=lambda key: on_release(key, pressed_keys),
    )
    listener.start()

    try:
        while True:
            state = KeyboardState()
            for key in keys_to_state:
                state.set_state(keys_to_state[key], 0)
                if is_key_pressed(key, pressed_keys):
                    state.set_state(keys_to_state[key], 1)
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
