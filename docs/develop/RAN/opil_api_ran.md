# RAN APIs 

It is possible to interact with the RAN via REST calls. With these calls it is possible to add, update or cancel tasks and assignments or access data. 

All the shown calls use the OCB APIs, that are exploted by passing an appropriate body message. 

An important element is thus the IP of the machine on which the Fiware Orion Context Broker is running - it is indicated as <b>OCB_IP</b>

All the highlighted attributes are detailed [here](../interfaces)

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


## GET Status

With this method it is possible to retieve informations about one enity. This might be a Robot Status, a RAN Status etc. For the whole list, look [here](../interfaces)

	GET OCB_IP:1026/v2/entities/#Name of the entity of interest#

## Other services

All of the previosuly presented calls are derived from Orion Context Broker APIs, that are documented in [here](https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#query-entity). It is possible to take advantage of all the methods that are listed in the documentation available at the provided link.

