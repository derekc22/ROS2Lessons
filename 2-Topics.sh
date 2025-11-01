# There are 2 types of nodes - Publisher and Subscriber
# A Publisher node sends messages to a topic
# A Subscriber node receives messages from a topic

ros2 run turtlesim turtlesim_node
# this node must be running for the following commands to work

ros2 run turtlesim turtle_teleop_key # run a teleop node to control the turtle via keyboard

ros2 topic list
# list currently active topics via the 'list' command

ros2 topic list -t
# list currently active topics with types via the 'list -t' command
# types gives you more information about the topic, including the message type

rqt_graph
# visualize the nodes and topics via the 'rqt_graph' command
# because 'turtlesim' and 'turtle_teleop_key' nodes are running, you will see them in the graph
# three topics will be visible: /turtle1/cmd_vel, /turtle1/rotate_absolute/_action/feedback, and /turtle1/rotate_absolute/_action/status
# respectively, these topics are used to control the turtle's velocity, provide feedback on rotation actions, and report the status of rotation actions

# /teleop_turtle sends velocity commands to the /turtle1/cmd_vel topic (a Publisher for this topic)
# /turtlesim receives velocity commands from the /turtle1/cmd_vel topic and moves the turtle accordingly (a Subscriber for this topic)

# /turtlesim sends feedback on rotation actions to the /turtle1/rotate_absolute/_action/feedback topic and reports the status of rotation actions to the /turtle1/rotate_absolute/_action/status topic (a Publisher for these topics)
# /teleop_turtle receives feedback and status updates from these topics (a Subscriber for these topics)

ros2 topic echo /turtle1/cmd_vel
# echo the messages being sent to the /turtle1/cmd_vel topic via the 'echo' command
# you will see the linear and angular velocity values being published to this topic as you control the turtle with the keyboard

ros2 topic info /turtle1/cmd_vel
# list information about the /turtle1/cmd_vel topic via the 'info' command
# this information includes the topic type (geometry_msgs/msg/Twist), the number of publishers (1), and the number of subscribers (1)

ros2 interface show geometry_msgs/msg/Twist
# show the structure of the message type used by the /turtle1/cmd_vel topic
# 'interface show' command is used to display the details of a specific message type
# 'geometry_msgs/msg/Twist' is the message type used by the /turtle1/cmd_vel topic
# this message type contains two main components: linear and angular, each of which has three fields (x, y, z) representing the velocity in 3D space

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
# publish a single message to the /turtle1/cmd_vel topic via the 'pub' command
# '--once' flag indicates that the message should be published only once
# 'geometry_msgs/msg/Twist' specifies the type of message being published
# the message being published sets the linear velocity in the x direction to 2.0 (moving forward) and the angular velocity around the z axis to 1.0 (rotating)

ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
# publish messages to the /turtle1/cmd_vel topic at a rate of 1 Hz via the 'pub' command
# '--rate 1' flag indicates that the message should be published at a frequency of 1 message per second

ros2 topic echo /turtle1/pose
# echo the messages being sent to the /turtle1/pose topic via the 'echo' command
# you will see the turtle's position (x, y), orientation (theta), linear velocity, and angular velocity being published to this topic as the turtle moves
# the /turtle1/pose topic is published by the turtlesim node and provides real-time updates on the turtle's state

ros2 topic hz /turtle1/pose
# monitor the frequency of messages being published to the /turtle1/pose topic via the 'hz' command
# this command will display the rate at which messages are being received on this topic, which is useful for understanding how often the turtle's state is being updated
# the output will show the average rate (in Hz), the minimum and maximum intervals between messages, and the standard deviation of the intervals