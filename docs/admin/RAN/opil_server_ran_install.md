# RAN Installation
 The RAN can be installed following 2 procedures, one using a Docker image, the other starting from the source code

## Docker

The Docker version does not need any installation or configuration, everything is resolved with the run command

	docker run '[...]' l4ms/opil.iot.ran:2.0

This command is explained in details in the [starting of RAN](../../user/introduction) , where all the required parameters will be explained

## Source code

Create the folder opil_v1 into the ROS workspace with catkin - it will appear under catkin_ws.
In order to do so, you can run the following command once you are into catkin_ws/src:

	catkin_create_pkg opil_v1
	
Copy the content of the source folder into the newly created opil_v1 folder.

Open a terminal on the workspace directory, type catkin_make and submit.

The repository contains various folders: config, FIROS config, include, map, msg, src plus some files.

- The config folder contains the configuration of various navigation parameters (like the costmaps used for the robot navigation).

- FIROS config folder contains the file for the configuration of FIROS – so the communication between the RAN and the Orion Context Broker.

- The include folder is just automatically created by catkin.

- The map folder contains the definition and configuration of the simulation environment.

- The msg folder contains the definition of the messages that are exchanged.

- The src folder contains the source code for the module.

- The amcl_config.xml file contains the parameters for the proper configuration of the navigation stack.

- CMakeList and package files define the structure of the ROS package. They are used when catkin builds the project.

- The .launch file define the whole starting procedure, setting all the necessary connections between all various ROS packages used in the module.
