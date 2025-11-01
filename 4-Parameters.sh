# Parameters are configuration values that can be set at runtime to modify the behavior of nodes
# Parameters can be used to adjust settings such as robot speed, sensor thresholds, or any other configurable option
# Parameters can be set and retrieved using the ROS 2 command line interface (CLI) or programmatically within a node
# Parameters can be dynamically changed while the node is running, allowing for flexible and adaptive behavior


ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
# these nodes must be running for the following commands to work

ros2 param list
# list currently set parameters via the 'list' command
# this will show parameters for all active nodes

ros2 param list /turtlesim
# list currently set parameters for a specific node via the 'list' command
# '/turtlesim' is the name of the node whose parameters you want to list
# this will show parameters specifically for the turtlesim node, including background color, pen color, and pen width

ros2 param get /turtlesim background_g
# get the value of a specific parameter via the 'get' command
# '/turtlesim' is the name of the node whose parameter you want to get
# 'background_g' is the name of the parameter whose value you want to retrieve
# this command will return the current value of the background_g parameter, which controls the green component of the background color in the turtlesim window

ros2 param set /turtlesim background_g 255
# set the value of a specific parameter via the 'set' command
# '/turtlesim' is the name of the node whose parameter you want to set
# 'background_g' is the name of the parameter whose value you want to change
# '255' is the new value to set for the background_g parameter
# after running this command, you should see the background color of the turtlesim window change to reflect the new green component value

ros2 param dump /turtlesim
# print all values of parameters for a specific node via the 'dump' command

ros2 param dump /turtlesim > turtlesim_params.yaml
# dump all parameter values for a specific node to a YAML file via the 'dump' command
# '/turtlesim' is the name of the node whose parameters you want to dump
# '>' is used to redirect the output to a file
# 'turtlesim_params.yaml' is the name of the file where the parameters will be saved
# this file can be used later to load the parameters back into the node

ros2 param load /turtlesim turtlesim_params.yaml
# load parameter values from a YAML file into a specific node via the 'load' command
# '/turtlesim' is the name of the node where the parameters will be loaded
# 'turtlesim_params.yaml' is the name of the file containing the parameter values to be loaded
# this command will set the parameters of the turtlesim node to the values specified in the YAML file

ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_params.yaml
# start a node with parameters loaded from a YAML file via the '--params-file' argument
# 'turtlesim_node' is the name of the node to run
# '--ros-args' indicates that the following arguments are specific to ROS 2
# '--params-file turtlesim_params.yaml' specifies the YAML file from which to load the parameters
# this command will start the turtlesim node with the parameters set according to the values in the specified YAML file