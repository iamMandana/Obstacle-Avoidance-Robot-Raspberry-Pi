# obstacle-avoidance-robot
The robot starts moving forward and checking for any obstacle on its way, if it detects any, it stops and turns right. This code is for Raspberry Pi.

Features:
- Continuous forward movement until an obstacle is detected
- Obstacle detection using an ultrasonic sensor
- Automatic stopping and right turn upon detection
- Simple logic for real-time obstacle avoidance

The avoid-obstacle client sends a start or stop commands to the avoid-obstacle service. Once the service recieves the start command, the motors start running and the ultrasonic sensor consistently check for any abstacle.
