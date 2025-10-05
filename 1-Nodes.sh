sudo apt install ros-humble-turtlesim
# install a ros2 package via 'sudo apt install'

ros2 run turtlesim turtlesim_node
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
# run a ros2 node via the 'run' command

ros2 node list
# list currently running nodes via the 'list' command

ros2 node info /turtlesim
# list information about currently running nodes via the 'node info' command
# these parameters include Subscribers, Publishers, Service Servers, etc.