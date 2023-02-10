# CDPR: ROS2 Controller

## User node
1. Get position (x, y) from stdin
2. Publish to topic "desired_pos"

## Robot node
1. Subscribe and receive positions (x, y) from topic "desired_pos"
2. Calculate trajectory to received robot position
3. Initialize time t=0
4. Calculate motor positions for right and left motors for a point in trajectory in time t
5. Publish motor positions to topics "motor_left_pos" and "motor_right_pos"
6. Increment time t: IF trajectory completed THEN finish ELSE go to step 4

## Motor node
1. Initialize motors interfaces and communication with drivers
2. Subscribe and receive data from topics "motor_left_pos" and "motor_right_pos"
3. Receive motor positions qRight and qLeft
4. Send command with positions qRight and qLeft to right and left motors respectively


