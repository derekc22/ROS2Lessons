# Whereas topics are used to continuously stream data between nodes, services are used for request/reply interactions.
# A node can offer a service (Service Server) that other nodes can call (Service Client).
# When a client calls a service, it sends a request message to the server, which processes the request and sends back a response message.
# Services are useful for operations that need a response, such as querying the state of a robot or commanding it to perform a specific action.
# If no call is made to a service, no data is transmitted, making services more efficient for certain types of interactions compared to topics.


ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
# these nodes must be running for the following commands to work

ros2 service list
# list currently active services via the 'list' command

ros2 service list -t
# list currently active services with types via the 'list -t' command
# types gives you more information about the service, including the request and response message types

ros2 service find std_srvs/srv/Empty
# find services of a specific type via the 'find' command
# 'std_srvs/srv/Empty' is a standard service type that takes no request data and returns no response data
# this service type is often used for simple commands, such as resetting a simulation or triggering an action without needing to send any additional information
# you should see the /reset and /clear services listed, which are used to reset the turtlesim simulation and clear the background, respectively

ros2 interface show turtlesim/srv/Spawn
# show the structure of the request and response messages for a specific service type via the 'interface show' command
# 'turtlesim/srv/Spawn' is a service type used to spawn a new turtle in the turtlesim simulation
# the request message contains fields for the turtle's x and y coordinates, theta (orientation), and name
# the response message contains a single field for the name of the spawned turtle

ros2 service call /clear std_srvs/srv/Empty
# call a service via the 'call' command
# '/clear' is the name of the service to call
# 'std_srvs/srv/Empty' is the type of the service being called
# since the Empty service type does not require any request data, no additional arguments are needed
# calling this service in particular will clear the background of the turtlesim window

ros2 service call /spawn turtlesim/srv/Spawn
# call the /spawn service to create a new turtle in the turtlesim simulation
# 'turtlesim/srv/Spawn' is the type of the service being called
# after calling this service, you should see a new turtle appear in the turtlesim window
# because no arguments were provided, the new turtle will be spawned at the default location (x: 0.0, y: 0.0) with the default name ('turtle2') (the turtle number is auto-incremented)

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
# call the /spawn service to create a new turtle in the turtlesim simulation
# 'turtlesim/srv/Spawn' is the type of the service being called
# here the request message includes the x and y coordinates (5.0, 5.0), theta (0.0), and name ('turtle2') for the new turtle
# after calling this service, you should see a new turtle appear in the turtlesim window at the specified location
# the response message will confirm the name of the spawned turtle