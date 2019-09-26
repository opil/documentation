# RAN APIs

It is possible to interact with the RAN via REST calls. With these calls it is possible to add, update or cancel tasks and assignments or access data.

All the shown calls use the OCB APIs, that are exploted by passing an appropriate body message.

An important element is thus the IP of the machine on which the Fiware Orion Context Broker is running - it is indicated as <b>OCB_IP</b>

All the highlighted attributesare detailed [here](../interfaces)

## POST Motion

With this, it is possible to set a new target for the robot (sending a new assignment) or to update an existing one.

	{
			"name": "postMotionv2",
			"request": {
				"method": "POST",
				"header": [
					{
						"key": "Content-Type",
						"name": "Content-Type",
						"value": "application/json",
						"type": "text"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"motion_channel\":\n\t{\n    \"type\": \"opil_v2.MotionAssignment\",\n    \"value\": {\n        \"use_orientation\": {\n            \"type\": \"boolean\",\n            \"value\": true\n        },\n        \"is_waypoint\": {\n            \"type\": \"boolean\",\n            \"value\": false\n        },\n        \"task_id\": {\n            \"type\": \"opil_v2.Id\",\n            \"value\": {\n                \"id\": {\n                    \"type\": \"number\",\n                    \"value\": 1\n                },\n                \"description\": {\n                    \"type\": \"string\",\n                    \"value\": \"\"\n                }\n            }\n        },\n        \"sequence\": {\n            \"type\": \"opil_v2.Sequence\",\n            \"value\": {\n                \"length\": {\n                    \"type\": \"number\",\n                    \"value\": 2\n                },\n                \"sequence_number\": {\n                    \"type\": \"number\",\n                    \"value\": 2\n                }\n            }\n        },\n        \"point_id\": {\n            \"type\": \"opil_v2.Id\",\n            \"value\": {\n                \"id\": {\n                    \"type\": \"number\",\n                    \"value\": 1\n                },\n                \"description\": {\n                    \"type\": \"string\",\n                    \"value\": \"\"\n                }\n            }\n        },\n        \"point\": {\n            \"type\": \"geometry_msgs.Pose2D\",\n            \"value\": {\n                \"y\": {\n                    \"type\": \"number\",\n                    \"value\": 13.5\n                },\n                \"x\": {\n                    \"type\": \"number\",\n                    \"value\": 19.5\n                },\n                \"theta\": {\n                    \"type\": \"number\",\n                    \"value\": 0.0\n                }\n            }\n        },\n        \"max_velocity\": {\n            \"type\": \"geometry_msgs.Twist\",\n            \"value\": {\n                \"linear\": {\n                    \"type\": \"geometry_msgs.Vector3\",\n                    \"value\": {\n                        \"y\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"x\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"z\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"angular\": {\n                    \"type\": \"geometry_msgs.Vector3\",\n                    \"value\": {\n                        \"y\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"x\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"z\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                }\n            }\n        },\n        \"header\": {\n            \"type\": \"std_msgs.Header\",\n            \"value\": {\n                \"stamp\": {\n                    \"type\": \"Time\",\n                    \"value\": {\n                        \"secs\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"nsecs\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"frame_id\": {\n                    \"type\": \"string\",\n                    \"value\": \"/map\"\n                },\n                \"seq\": {\n                    \"type\": \"number\",\n                    \"value\": 0\n                }\n            }\n        },\n        \"motion_area\": {\n            \"type\": \"geometry_msgs.PolygonStamped\",\n            \"value\": {\n                \"header\": {\n                    \"type\": \"std_msgs.Header\",\n                    \"value\": {\n                        \"stamp\": {\n                            \"type\": \"Time\",\n                            \"value\": {\n                                \"secs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 0\n                                },\n                                \"nsecs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 0\n                                }\n                            }\n                        },\n                        \"frame_id\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        },\n                        \"seq\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"polygon\": {\n                    \"type\": \"geometry_msgs.Polygon\",\n                    \"value\": {\n                        \"points\": {\n                            \"type\": \"array\",\n                            \"value\": []\n                        }\n                    }\n                }\n            }\n        },\n        \"motion_id\": {\n            \"type\": \"opil_v2.Id\",\n            \"value\": {\n                \"id\": {\n                    \"type\": \"number\",\n                    \"value\": 2\n                },\n                \"description\": {\n                    \"type\": \"string\",\n                    \"value\": \"\"\n                }\n            }\n        },\n        \"robot_id\": {\n            \"type\": \"opil_v2.Id\",\n            \"value\": {\n                \"id\": {\n                    \"type\": \"number\",\n                    \"value\": 0\n                },\n                \"description\": {\n                    \"type\": \"string\",\n                    \"value\": \"\"\n                }\n            }\n        },\n        \"max_acceleration\": {\n            \"type\": \"geometry_msgs.Accel\",\n            \"value\": {\n                \"linear\": {\n                    \"type\": \"geometry_msgs.Vector3\",\n                    \"value\": {\n                        \"y\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"x\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"z\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"angular\": {\n                    \"type\": \"geometry_msgs.Vector3\",\n                    \"value\": {\n                        \"y\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"x\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"z\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                }\n            }\n        }\n    },\n    \"metadata\": {\n        \"dataType\": {\n            \"type\": \"dataType\",\n            \"value\": {\n                \"use_orientation\": \"bool\",\n                \"is_waypoint\": \"bool\",\n                \"task_id\": {\n                    \"id\": \"uint32\",\n                    \"description\": \"string\"\n                },\n                \"sequence\": {\n                    \"length\": \"int32\",\n                    \"sequence_number\": \"int32\"\n                },\n                \"point_id\": {\n                    \"id\": \"uint32\",\n                    \"description\": \"string\"\n                },\n                \"point\": {\n                    \"y\": \"float64\",\n                    \"x\": \"float64\",\n                    \"theta\": \"float64\"\n                },\n                \"max_velocity\": {\n                    \"linear\": {\n                        \"y\": \"float64\",\n                        \"x\": \"float64\",\n                        \"z\": \"float64\"\n                    },\n                    \"angular\": {\n                        \"y\": \"float64\",\n                        \"x\": \"float64\",\n                        \"z\": \"float64\"\n                    }\n                },\n                \"header\": {\n                    \"stamp\": {\n                        \"secs\": \"int32\",\n                        \"nsecs\": \"int32\"\n                    },\n                    \"frame_id\": \"string\",\n                    \"seq\": \"uint32\"\n                },\n                \"motion_area\": {\n                    \"header\": {\n                        \"stamp\": {\n                            \"secs\": \"int32\",\n                            \"nsecs\": \"int32\"\n                        },\n                        \"frame_id\": \"string\",\n                        \"seq\": \"uint32\"\n                    },\n                    \"polygon\": {\n                        \"points\": \"geometry_msgs/Point32[]\"\n                    }\n                },\n                \"motion_id\": {\n                    \"id\": \"uint32\",\n                    \"description\": \"string\"\n                },\n                \"robot_id\": {\n                    \"id\": \"uint32\",\n                    \"description\": \"string\"\n                },\n                \"max_acceleration\": {\n                    \"linear\": {\n                        \"y\": \"float64\",\n                        \"x\": \"float64\",\n                        \"z\": \"float64\"\n                    },\n                    \"angular\": {\n                        \"y\": \"float64\",\n                        \"x\": \"float64\",\n                        \"z\": \"float64\"\n                    }\n                }\n            }\n        }\n    }\n}\n}"
				},
				"url": {
					"raw": "FIWAREHOST:1026/v2/entities/robot_opil_v2/attrs",
					"host": [
						"FIWAREHOST"
					],
					"port": "1026",
					"path": [
						"v2",
						"entities",
						"robot_opil_v2",
						"attrs"
					]
				}
			},
			"response": []
		},
		
				
Relevant attributes of the body are:

- task_id 

- motion_id

- point 

the definition of the target, with coordinates

- sequence

put the motion in the correct sequence position



## POST Action

With this, it is possible to set a new action for the robot to perform (sending a new assignment) or to update an existing one.

	{
			"name": "postActionv2",
			"request": {
				"method": "POST",
				"header": [
					{
						"key": "Content-Type",
						"name": "Content-Type",
						"value": "application/json",
						"type": "text"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"action_channel\": {\n            \"type\": \"opil_v2.ActionAssignment\",\n            \"value\": {\n                \"task_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 1\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                },\n                \"sequence\": {\n                    \"type\": \"opil_v2.Sequence\",\n                    \"value\": {\n                        \"length\": {\n                            \"type\": \"number\",\n                            \"value\": 1\n                        },\n                        \"sequence_number\": {\n                            \"type\": \"number\",\n                            \"value\": 1\n                        }\n                    }\n                },\n                \"header\": {\n                    \"type\": \"std_msgs.Header\",\n                    \"value\": {\n                        \"stamp\": {\n                            \"type\": \"Time\",\n                            \"value\": {\n                                \"secs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 1552905495\n                                },\n                                \"nsecs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 59420918\n                                }\n                            }\n                        },\n                        \"frame_id\": {\n                            \"type\": \"string\",\n                            \"value\": \"/map\"\n                        },\n                        \"seq\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"robot_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                },\n                \"action\": {\n                    \"type\": \"opil_v2.RobotAction\",\n                    \"value\": {\n                        \"category\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"action\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        },\n                        \"attributes\": {\n                            \"type\": \"array\",\n                            \"value\": []\n                        }\n                    }\n                },\n                \"action_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 1\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                }\n            },\n            \"metadata\": {\n                \"dataType\": {\n                    \"type\": \"dataType\",\n                    \"value\": {\n                        \"task_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        },\n                        \"sequence\": {\n                            \"length\": \"int32\",\n                            \"sequence_number\": \"int32\"\n                        },\n                        \"header\": {\n                            \"stamp\": {\n                                \"secs\": \"int32\",\n                                \"nsecs\": \"int32\"\n                            },\n                            \"frame_id\": \"string\",\n                            \"seq\": \"uint32\"\n                        },\n                        \"robot_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        },\n                        \"action\": {\n                            \"category\": \"uint8\",\n                            \"action\": \"uint8\",\n                            \"description\": \"string\",\n                            \"attributes\": \"opil_v2/Tuple[]\"\n                        },\n                        \"action_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        }\n                    }\n                }\n            }\n        }\n}"
				},
				"url": {
					"raw": "FIWAREHOST:1026/v2/entities/robot_opil_v2/attrs",
					"host": [
						"FIWAREHOST"
					],
					"port": "1026",
					"path": [
						"v2",
						"entities",
						"robot_opil_v2",
						"attrs"
					]
				}
			},
			"response": []
		},
		
				
The relevant attibutes are:

- task_id

- action_id

- sequence

## POST Cancel Assignmet or Task

With this, it is possible to set a cancel any assignment or a whole task.

	{
			"name": "cancelTaskv2",
			"request": {
				"method": "POST",
				"header": [
					{
						"key": "Content-Type",
						"value": "application/json",
						"type": "text"
					}
				],
				"body": {
					"mode": "raw",
					"raw": "{\"task_management_channel\": {\n            \"type\": \"opil_v2.CancelTask\",\n            \"value\": {\n                \"robot_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                },\n                \"header\": {\n                    \"type\": \"std_msgs.Header\",\n                    \"value\": {\n                        \"stamp\": {\n                            \"type\": \"Time\",\n                            \"value\": {\n                                \"secs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 1552915175\n                                },\n                                \"nsecs\": {\n                                    \"type\": \"number\",\n                                    \"value\": 93503735\n                                }\n                            }\n                        },\n                        \"frame_id\": {\n                            \"type\": \"string\",\n                            \"value\": \"/map\"\n                        },\n                        \"seq\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        }\n                    }\n                },\n                \"action_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 0\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                },\n                \"task_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 1\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                },\n                \"motion_id\": {\n                    \"type\": \"opil_v2.Id\",\n                    \"value\": {\n                        \"id\": {\n                            \"type\": \"number\",\n                            \"value\": 2\n                        },\n                        \"description\": {\n                            \"type\": \"string\",\n                            \"value\": \"\"\n                        }\n                    }\n                }\n            },\n            \"metadata\": {\n                \"dataType\": {\n                    \"type\": \"dataType\",\n                    \"value\": {\n                        \"robot_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        },\n                        \"header\": {\n                            \"stamp\": {\n                                \"secs\": \"int32\",\n                                \"nsecs\": \"int32\"\n                            },\n                            \"frame_id\": \"string\",\n                            \"seq\": \"uint32\"\n                        },\n                        \"action_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        },\n                        \"task_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        },\n                        \"motion_id\": {\n                            \"id\": \"uint32\",\n                            \"description\": \"string\"\n                        }\n                    }\n                }\n            }\n        }\n}"
				},
				"url": {
					"raw": "localhost:1026/v2/entities/robot_opil_v2/attrs",
					"host": [
						"localhost"
					],
					"port": "1026",
					"path": [
						"v2",
						"entities",
						"robot_opil_v2",
						"attrs"
					]
				}
			},
			"response": []
		},
		
				
The relevant attibutes are:

- task_id

- motion_id

- action_id


## GET Status

With this method it is possible to retieve informations about one enity. This might be a Robot Status, a RAN Status etc. For the whole list, look [here](../interfaces)

	GET FIWAREHOST:1026/v2/entities/#Name of the entity of interest#
	
	example:
	
	{
			"name": "getEntitiesv2",
			"request": {
				"method": "GET",
				"header": [],
				"body": {
					"mode": "raw",
					"raw": ""
				},
				"url": {
					"raw": "FIWAREHOST:1026/v2/entities/robot_opil_v2/",
					"host": [
						"FIWAREHOST"
					],
					"port": "1026",
					"path": [
						"v2",
						"entities",
						"robot_opil_v2",
						""
					]
				}
			},
			"response": []
		}

## Other services

All of the previosuly presented calls are derived from Orion Context Broker APIs, that are documented in [here](https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#query-entity). It is possible to take advantage of all the methods that are listed in the documentation availabe at the provided link.
