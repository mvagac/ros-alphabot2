# AlphaBot2

Must by run as root user..

ros2 launch src/alphabot2/launch/alphabot2.py

ros2 topic pub -1 /diff_controller_alphabot2/cmd_vel geometry_msgs/msg/TwistStamped "{ header: {frame_id: ’/telo’}, twist: { linear: {x: 2.0}, angular: {z: 1.8}}}"
