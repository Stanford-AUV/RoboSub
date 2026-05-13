See README.md in this folder for full documentation.

Quick start:
  ./build.sh && source install/setup.bash
  ros2 run gui bridge
  ros2 launch main manual.py   # if you need teleop / thrust stack
  Open gui/auv_hud.html — WebSocket defaults to ws://localhost:9091 (see ros2_gui_bridge.py).
