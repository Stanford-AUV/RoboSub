export GZ_PARTITION=127.0.0.1:ros
gz sim src/simulation/simulation/models/world.urdf -s -r \
& ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/simulation/simulation/gazebo_bridge.yaml \
& ros2 launch src/launch/simulation.py