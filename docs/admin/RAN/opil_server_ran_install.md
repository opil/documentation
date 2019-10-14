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

<!---
#Getting Started
-->

# RAN Deinstall


## Docker

In order to unstall the RAN, if you're using Docker, just follow the same procedure for deinstalling any other container. 

A guide can be found [here](https://docs.docker.com/engine/reference/commandline/rm/)

## Source code

In order to uninstall the RAN, if you're using the Source code version, you will need to remove the whole opil package. It is a really simple procedure. It i enought to delete the package folder and then build again with catkin_make. 

# RAN Upgrades

Every upgrade of the RAN module will be released in both the Docker version and as Source code. 

##Docker

In order to use the new Docker Image, change the version number at the end of the "run" command. The new image will be downloaded and the new instance will contain the new, updated version.

##Source code

In order to use the upgraded version of the sorce code, download the new version of the code and substitute it inside the package, then run "catkin_make"




# RAN Deprecated Functionalities

From RANv4 on, the FIROS component had been updated from [firos](https://github.com/Ikergune/firos) to [firos"++"](https://github.com/iml130/firos)

For this reason, the following API calls are deprecated, and only works for RAN versions 1 to 3

## POST Motion

With this, it is possible to set a new target for the robot (sending a new assignment) or to update an existing one.

	POST OCB_IP:1026/v1/updateContext
				"header": [
					{
						"key": "Content-Type",
						"value": "application/json"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"contextElements\": [{\"attributes\": [{\"type\": \"COMMAND\", \"name\": \"COMMAND\", \"value\": [\"motion_channel\"]}, {\"type\": \"opil_v1.msg.MotionAssignment\", \"name\": \"motion_channel\", \"value\":

					\"{%27is_waypoint%27: false,

					%27task_id%27: {%27id%27: 1, %27description%27: %27%27},

					%27point%27: {%27y%27: 13.5, %27x%27: 11.0, %27theta%27: 0.0}, %27point_id%27: {%27id%27: 1, %27description%27: %27%27}, 
					
					%27sequence%27: {%27length%27: 2, %27sequence_number%27:1},

					%27motion_area%27: {%27header%27: {%27stamp%27: {%27secs%27: 0, %27nsecs%27: 0}, %27frame_id%27: %27%27, %27seq%27: 0}, %27polygon%27: {%27points%27: []}},

					%27max_velocity%27: {%27linear%27: {%27y%27: 0.0, %27x%27: 0.0, %27z%27: 0.0}, %27angular%27: {%27y%27: 0.0, %27x%27: 0.0, %27z%27: 0.0}},

					%27header%27: {%27stamp%27: {%27secs%27: 1536222372, %27nsecs%27: 930678771}, %27frame_id%27: %27/map%27, %27seq%27: 0},

					%27max_acceleration%27: {%27linear%27: {%27y%27: 0.0, %27x%27: 0.0, %27z%27: 0.0}, %27angular%27: {%27y%27: 0.0, %27x%27: 0.0, %27z%27: 0.0}},

					%27motion_id%27: {%27id%27: 1, %27description%27: %27%27},

					%27robot_id%27: {%27id%27: 0, %27description%27: %27%27}, 
					
					%27use_orientation%27: true}\"
					
					}], \"type\": \"ROBOT\", \"id\": \"robot_opil_v1\", \"isPattern\": \"false\"}], \"updateAction\": \"APPEND\"}"
				}
				
Relevant attributes of the body are:

- task_id 

- motion_id

- point 

the definition of the target, with coordinates

- sequence

put the motion in the correct sequence position



## POST Action

With this, it is possible to set a new action for the robot to perform (sending a new assignment) or to update an existing one.

	POST OCB_IP:1026/v1/updateContext
	
				"header": [
					{
						"key": "Content-Type",
						"value": "application/json"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"contextElements\": [{\"attributes\": [{\"type\": \"COMMAND\", \"name\": \"COMMAND\", \"value\": [\"action_channel\"]}, {\"type\": \"opil_v1.msg.ActionAssignment\", \"name\": \"action_channel\", \"value\": 
					
					\"{%27firosstamp%27: 1540469184.269263,

					%27task_id%27: {%27id%27: 1, %27description%27: %27%27},

					%27sequence%27: {%27length%27: 1, %27sequence_number%27: 1},

					%27header%27: {%27stamp%27: {%27secs%27: 1540469164, %27nsecs%27: 146275735}, %27frame_id%27: %27/map%27, %27seq%27: 0},

					%27robot_id%27: {%27id%27: 0, %27description%27: %27%27}, 
					
					%27action%27: 1, 
					
					%27action_id%27: {%27id%27: 1, %27description%27: %27%27}}
					
					\"}], \"type\": \"ROBOT\", \"id\": \"robot_opil_v1\", \"isPattern\": \"false\"}], \"updateAction\": \"APPEND\"}"
				}
				
The relevant attibutes are:

- task_id

- action_id

- sequence

## POST Cancel Assignmet or Task

With this, it is possible to set a cancel any assignment or a whole task.

	POST OCB_IP:1026/v1/updateContext
	
				"header": [
					{
						"key": "Content-Type",
						"value": "application/json"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"contextElements\": [{\"attributes\": [{\"type\": \"COMMAND\", \"name\": \"COMMAND\", \"value\": [\"task_management_channel\"]},

					{\"type\": \"opil_v1.msg.CancelTask\", \"name\": \"task_management_channel\", \"value\":

					\"{%27task_id%27: {%27id%27: 1, %27description%27: %27%27},

					%27header%27: {%27stamp%27: {%27secs%27: 1540384824, %27nsecs%27: 313047175}, %27frame_id%27: %27/map%27, %27seq%27: 1},

					%27motion_id%27: {%27id%27: 2, %27description%27: %27%27},

					%27robot_id%27: {%27id%27: 0, %27description%27: %27%27},

					%27action_id%27: {%27id%27: 0, %27description%27: %27%27}}\"}],

					\"type\": \"ROBOT\", \"id\": \"robot_opil_v1\", \"isPattern\": \"false\"}], \"updateAction\": \"APPEND\"}"
				}
				
The relevant attibutes are:

- task_id

- motion_id

- action_id






