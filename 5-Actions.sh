# Like Services, Actions also have a client-server architecture
# However, Actions have 'Goals' 'Feedback' and 'Results', making them more complex than Services

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
# these nodes must be running for the following commands to work

ros2 node info /teleop_turtle
# list information about the /teleop_turtle node via the 'node info' command
# under the 'Action Clients' section, you should see the /turtle1/rotate_absolute action listed
# this action is used to rotate the turtle to a specific angle
# using the turtle_teleop_key node, you can rotate the turtle by pressing the G|B|V|C|D|E|R|T keys
# if the turtle is successfully rotated, you  will see '[INFO] [1760150183.034107793] [turtlesim]: Rotation goal completed successfully'
# if you interrupt the rotation by pressing a different rotation key, you will see '[WARN] [1760150170.122290604] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal'
# if you cancel the rotation with the 'F', you will see '[INFO] [1760150379.274214644] [turtlesim]: Rotation goal canceled'

ros2 action list
# list currently active actions via the 'list' command
# you should see the /turtle1/rotate_absolute action listed

ros2 action list -t
# list currently active actions with types via the 'list -t' command
# types gives you more information about the action, including the goal, feedback, and result message types

ros2 action info turtle1/rotate_absolute
# list information about a specific action via the 'info' command
# 'turtle1/rotate_absolute' is the name of the action whose information you want to list
# this information includes the action client (teleop_turtle) and action server (turtlesim)

ros2 interface show turtlesim/action/RotateAbsolute
# to see the datatype for the action, use the 'interface show' command
# 'turtlesim/action/RotateAbsolute' is the action type used by the /turtle1/rotate_absolute action
# this action type contains three main components: Goal, Feedback, and Result
# the Goal message contains a field for 'theta', which is 'the desired heading in radians'
# the Feedback message contains a field for 'delta', which is 'the angular displacement in radians to the starting position'
# the Result message contains a field for 'remaining', which is 'the remaining rotation in radians' b