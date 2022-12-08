# ROS2 Controller

## User node
1. Get position (x, y) from stdin
2. Publish to topic "robot_pos"

## Motor node
1. Subscribe and receive data from topic "robot_pos"
2. Calculate trajectory to received robot position
3. Publish to topic "motor_pos" motor positions calculated at each incremented time

## Epos node
1. Subscribe and receive data from topic "motor_pos" with node id and motor position
2. Set position of motor in <<nodeId>> to position received


