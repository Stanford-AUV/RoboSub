### RUN LOCALLY ###

import pygame
import requests
import time

# --- Configuration ---
SERVER_URL = "http://localhost:8000/state"  # Change if remote
DEADZONE = 0.1  # Ignore tiny joystick noise
SEND_INTERVAL_MS = 50  # Send every 100 ms

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

# --- Main Loop ---
try:
    while True:
        pygame.event.pump()

        lx = get_axis(0)
        ly = get_axis(5)
        rx = get_axis(3)
        ry = get_axis(4)
        enabled = get_axis(1) < 0 and get_axis(7) < 0

        payload = {
            "lx": lx,
            "ly": ly,
            "rx": rx,
            "ry": ry,
            "enabled": enabled
        }

        try:
            res = requests.post(SERVER_URL, json=payload, timeout=0.3)
            print(f"Sent: {payload} -> {res.status_code}")
        except requests.RequestException as e:
            print("Failed to send:", e)

        time.sleep(SEND_INTERVAL_MS / 1000.0)

except KeyboardInterrupt:
    print("Exiting.")
finally:
    pygame.quit()
