./build.sh && source install/setup.bash

# In Terminal 1 — start the bridge
ros2 run gui bridge

# In Terminal 2 — launch manual joystick as normal
ros2 launch main manual.py

# Open auv_hud.html in your browser
# It auto-connects to ws://localhost:9090
