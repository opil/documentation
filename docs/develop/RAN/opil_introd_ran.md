# How to use the RAN module
How to start the RAN depends on how you get it and how you installed it. 

## How to start the RAN
### Docker
 If you are using Docker, to start the RAN, run the following commands on the terminal:
 
	xhost local:root
	
	docker run -e FIWAREHOST=x.x.x.x -e NETINTERFACE=name -- network host-e DISPLAY=$DISPLAY --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" l4ms/opil.iot.ran:4.0
	
	
Both solutions might require a ‘sudo’ prefix

The first command is used in order to give the container access to the host display. It is required for the running of stage – a ROS simulator – which requires the access to the display to show the movement of the robot in the simulated environment. This dependence from display will be removed in the next versions.

The other parameters create some evnvironment variables used in the Docker image.


The FIWAREHOST parameter has to be set to the IP of the machine on which the Orion Context Broker is running.

The NETINTERFACE is used to specify which network interface is used for communication. It is possible to get that name by using the ifconfig command


It is also possible to set up the environment via the docker-compose up command. In order to use it, you need to properly set up the parameters in the .yaml file, following the same procedure as explained above.


### Source code:
Using the source code, you just need to execute the <b>*.launch</b> file present in the Source folder. BE SURE FIWARE-OCB IS UP.

In order to launch the file, open the terminal and type: 

	roslaunch robot_launcher.launch
	
In the launch file it is possible to personalize and custom some aspects of the RAN. The most relevant information that has to be provided is the <b>AGV/Robot identifcator</b>, so the integer values that identifies the HW in the architecture. We recommend to use incremental integer numbers to identify each single machine.


## How to interact with the RAN

It is possible to interact with the RAN via REST calls. These calls allow to set a target for the robot to reach, to order the robot to perform an action or to modify or cancel an existing assignment. There are no dependence between the Calls and the starting method, so the procedure is the same for both Docker and Source code. These APIs are defined in [here](../api)
