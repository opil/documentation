# Interfaces

Subsequently a detailed description of the exchanged entites is given. 

## TP <-> RAN

The following listing depicts the complete exchanged messages between RAN and TP. Subsequently all key value pairs will be explained in detail.

```json
{
    "id": "robot_opil_v2",
    "type": "ROBOT",
    "action_assignment": {
     ... },
    "cancel_order": {
     ... },
    "current_motion": {
     ... },
    "motion_assignment": {
     ... },
    "robot_description": {
     ... }
}
```

### Interfaces produced by TP

In the following all messages will be explained which are send by the TP. Messages might be based on some non-primitive types (e.g. mars_common_msgs/Id and others); These types are explained at the end of this document ([Used messages inside TP messages](#used-messages-inside-tp-messages)).

#### MotionAssignment.msg

The motion assignment tells the AGV the next destionation and under which circumstances it can moves to this position.

| Type                         | Variable         | Description                                                                                                                                                                                                       |
| ---------------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | point_id         | ID of the next vertex / edge were the AGV should drive to.                                                                                                                                                        |
| mars_common_msgs/Id          | task_id          | ID of the task to which the MotionAssignment belongs.                                                                                                                                                             |
| mars_common_msgs/Id          | motion_id        | ID of the MotionAssignment. A new ID must be generated for each MotionAssignment.                                                                                                                                 |
| bool                         | is_waypoint      | TRUE if the point is a waypoint (intermediate point along the path), FALSE if it is a goal.                                                                                                                       |
| bool                         | use_orientation  | TRUE if the theta of the point has to be considered.                                                                                                                                                              |
| geometry_msgs/Twist          | max_velocity     | Maximum allowed velocity in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)            |
| geometry_msgs/Accel          | max_acceleration | Maximum allowed acceleration in the current segment. Segment is defined by the **motion_area**. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Accel.html) |
| geometry_msgs/PolygonStamped | motion_area      | Area in which the vehicle can move freely. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PolygonStamped.html)                                             |
| Sequence                     | sequence         | Sequence number of the current MotionAssignment.                                                                                                                                                                  |

##### ROS Message
```ini
Header header
mars_common_msgs/Id point_id
mars_common_msgs/Id task_id
mars_common_msgs/Id motion_id
geometry_msgs/Pose2D point
# TRUE if the point is a waypoint, FALSE if it is a goal
bool is_waypoint
# TRUE if the theta of the point has to be considered
bool use_orientation
geometry_msgs/Twist max_velocity
geometry_msgs/Accel max_acceleration
# defines the area in which the robot can move
geometry_msgs/PolygonStamped motion_area
# the position of the assignment in the sequence
Sequence sequence
```

##### Example of Orion entity

```json
"motion_assignment": {
    "type": "mars_agent_physical_robot_msgs.MotionAssignment",
    "value": {
        "use_orientation": {
            "type": "boolean",
            "value": false
        },
        "is_waypoint": {
            "type": "boolean",
            "value": true
        },
        "task_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 76
                        },
                        {
                            "type": "number",
                            "value": 228
                        },
                        {
                            "type": "number",
                            "value": 77
                        },
                        {
                            "type": "number",
                            "value": 190
                        },
                        {
                            "type": "number",
                            "value": 160
                        },
                        {
                            "type": "number",
                            "value": 186
                        },
                        {
                            "type": "number",
                            "value": 77
                        },
                        {
                            "type": "number",
                            "value": 49
                        },
                        {
                            "type": "number",
                            "value": 145
                        },
                        {
                            "type": "number",
                            "value": 55
                        },
                        {
                            "type": "number",
                            "value": 246
                        },
                        {
                            "type": "number",
                            "value": 239
                        },
                        {
                            "type": "number",
                            "value": 100
                        },
                        {
                            "type": "number",
                            "value": 23
                        },
                        {
                            "type": "number",
                            "value": 251
                        },
                        {
                            "type": "number",
                            "value": 73
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "sequence": {
            "type": "mars_agent_physical_robot_msgs.Sequence",
            "value": {
                "length": {
                    "type": "number",
                    "value": 20
                },
                "sequence_number": {
                    "type": "number",
                    "value": 20
                }
            }
        },
        "point_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 211
                        },
                        {
                            "type": "number",
                            "value": 15
                        },
                        {
                            "type": "number",
                            "value": 212
                        },
                        {
                            "type": "number",
                            "value": 98
                        },
                        {
                            "type": "number",
                            "value": 168
                        },
                        {
                            "type": "number",
                            "value": 77
                        },
                        {
                            "type": "number",
                            "value": 89
                        },
                        {
                            "type": "number",
                            "value": 64
                        },
                        {
                            "type": "number",
                            "value": 181
                        },
                        {
                            "type": "number",
                            "value": 51
                        },
                        {
                            "type": "number",
                            "value": 63
                        },
                        {
                            "type": "number",
                            "value": 72
                        },
                        {
                            "type": "number",
                            "value": 110
                        },
                        {
                            "type": "number",
                            "value": 11
                        },
                        {
                            "type": "number",
                            "value": 133
                        },
                        {
                            "type": "number",
                            "value": 97
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "point": {
            "type": "geometry_msgs.Pose2D",
            "value": {
                "y": {
                    "type": "number",
                    "value": -3.119999886
                },
                "x": {
                    "type": "number",
                    "value": -0.916000009
                },
                "theta": {
                    "type": "number",
                    "value": 0
                }
            }
        },
        "max_velocity": {
            "type": "geometry_msgs.Twist",
            "value": {
                "linear": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 10
                        },
                        "z": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "angular": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 0
                        },
                        "z": {
                            "type": "number",
                            "value": 1
                        }
                    }
                }
            }
        },
        "header": {
            "type": "std_msgs.Header",
            "value": {
                "stamp": {
                    "type": "Time",
                    "value": {
                        "secs": {
                            "type": "number",
                            "value": 1572008882
                        },
                        "nsecs": {
                            "type": "number",
                            "value": 381286634
                        }
                    }
                },
                "frame_id": {
                    "type": "string",
                    "value": ""
                },
                "seq": {
                    "type": "number",
                    "value": 19
                }
            }
        },
        "motion_area": {
            "type": "geometry_msgs.PolygonStamped",
            "value": {
                "header": {
                    "type": "std_msgs.Header",
                    "value": {
                        "stamp": {
                            "type": "Time",
                            "value": {
                                "secs": {
                                    "type": "number",
                                    "value": 0
                                },
                                "nsecs": {
                                    "type": "number",
                                    "value": 0
                                }
                            }
                        },
                        "frame_id": {
                            "type": "string",
                            "value": ""
                        },
                        "seq": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "polygon": {
                    "type": "geometry_msgs.Polygon",
                    "value": {
                        "points": {
                            "type": "array",
                            "value": [
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -2.170000076
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": 0.034000002
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -2.170000076
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": -1.866000056
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -4.070000172
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": -1.866000056
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -4.070000172
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": 0.034000002
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                }
                            ],
                            "metadata": {
                                "dataType": {
                                    "type": "dataType",
                                    "value": "geometry_msgs/Point32[]"
                                }
                            }
                        }
                    }
                }
            }
        },
        "motion_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 30
                        },
                        {
                            "type": "number",
                            "value": 175
                        },
                        {
                            "type": "number",
                            "value": 251
                        },
                        {
                            "type": "number",
                            "value": 36
                        },
                        {
                            "type": "number",
                            "value": 210
                        },
                        {
                            "type": "number",
                            "value": 224
                        },
                        {
                            "type": "number",
                            "value": 64
                        },
                        {
                            "type": "number",
                            "value": 42
                        },
                        {
                            "type": "number",
                            "value": 173
                        },
                        {
                            "type": "number",
                            "value": 87
                        },
                        {
                            "type": "number",
                            "value": 188
                        },
                        {
                            "type": "number",
                            "value": 44
                        },
                        {
                            "type": "number",
                            "value": 128
                        },
                        {
                            "type": "number",
                            "value": 109
                        },
                        {
                            "type": "number",
                            "value": 196
                        },
                        {
                            "type": "number",
                            "value": 11
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "max_acceleration": {
            "type": "geometry_msgs.Accel",
            "value": {
                "linear": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 1
                        },
                        "z": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "angular": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 0
                        },
                        "z": {
                            "type": "number",
                            "value": 0.5
                        }
                    }
                }
            }
        }
    },
    "metadata": {
        "dataType": {
            "type": "dataType",
            "value": {
                "use_orientation": "bool",
                "is_waypoint": "bool",
                "task_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "sequence": {
                    "length": "int32",
                    "sequence_number": "int32"
                },
                "point_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "point": {
                    "y": "float64",
                    "x": "float64",
                    "theta": "float64"
                },
                "max_velocity": {
                    "linear": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    },
                    "angular": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    }
                },
                "header": {
                    "stamp": {
                        "secs": "int32",
                        "nsecs": "int32"
                    },
                    "frame_id": "string",
                    "seq": "uint32"
                },
                "motion_area": {
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "polygon": {
                        "points": "geometry_msgs/Point32[]"
                    }
                },
                "motion_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "max_acceleration": {
                    "linear": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    },
                    "angular": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    }
                }
            }
        }
    }
}
```

#### ActionAssignment.msg

The action assignment tells the AGV which action has to be performed.

| Type                | Variable     | Description                                                                                          |
| ------------------- | ------------ | ---------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id | action_id    | ID of the ActionAssignment. A new ID must be generated for each ActionAssignment.                    |
| mars_common_msgs/Id | task_id      | ID of the task to which the ActionAssignment belongs.                                                |
| Sequence            | sequence     | Sequence number of the current MotionAssignment.                                                     |
| RobotAction         | robot_action | Action which should be performed. For more detailed description go to the Message description above. |

##### ROS Message

```ini
# Message for the definition of an Action
mars_common_msgs/Id action_id
mars_common_msgs/Id task_id
# Position of the action within the order
Sequence sequence
RobotAction robot_action

```

##### Example of Orion entity

```json
"action_assignment": {
    "type": "mars_agent_physical_robot_msgs.ActionAssignment",
    "value": {
        "robot_action": {
            "type": "mars_agent_physical_robot_msgs.RobotAction",
            "value": {
                "category": {
                    "type": "number",
                    "value": 10
                },
                "action": {
                    "type": "number",
                    "value": 5
                },
                "description": {
                    "type": "string",
                    "value": ""
                },
                "attributes": {
                    "type": "array",
                    "value": [
                        {
                            "type": "mars_common_msgs.Tuple",
                            "value": {
                                "type": {
                                    "type": "string",
                                    "value": ""
                                },
                                "name": {
                                    "type": "string",
                                    "value": ""
                                },
                                "value": {
                                    "type": "string",
                                    "value": ""
                                }
                            }
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "mars_common_msgs/Tuple[]"
                        }
                    }
                }
            }
        },
        "action_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 31
                        },
                        {
                            "type": "number",
                            "value": 54
                        },
                        {
                            "type": "number",
                            "value": 230
                        },
                        {
                            "type": "number",
                            "value": 47
                        },
                        {
                            "type": "number",
                            "value": 237
                        },
                        {
                            "type": "number",
                            "value": 27
                        },
                        {
                            "type": "number",
                            "value": 86
                        },
                        {
                            "type": "number",
                            "value": 34
                        },
                        {
                            "type": "number",
                            "value": 183
                        },
                        {
                            "type": "number",
                            "value": 123
                        },
                        {
                            "type": "number",
                            "value": 216
                        },
                        {
                            "type": "number",
                            "value": 30
                        },
                        {
                            "type": "number",
                            "value": 6
                        },
                        {
                            "type": "number",
                            "value": 182
                        },
                        {
                            "type": "number",
                            "value": 31
                        },
                        {
                            "type": "number",
                            "value": 11
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "task_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 126
                        },
                        {
                            "type": "number",
                            "value": 145
                        },
                        {
                            "type": "number",
                            "value": 110
                        },
                        {
                            "type": "number",
                            "value": 222
                        },
                        {
                            "type": "number",
                            "value": 168
                        },
                        {
                            "type": "number",
                            "value": 165
                        },
                        {
                            "type": "number",
                            "value": 90
                        },
                        {
                            "type": "number",
                            "value": 83
                        },
                        {
                            "type": "number",
                            "value": 141
                        },
                        {
                            "type": "number",
                            "value": 113
                        },
                        {
                            "type": "number",
                            "value": 29
                        },
                        {
                            "type": "number",
                            "value": 45
                        },
                        {
                            "type": "number",
                            "value": 22
                        },
                        {
                            "type": "number",
                            "value": 23
                        },
                        {
                            "type": "number",
                            "value": 1
                        },
                        {
                            "type": "number",
                            "value": 132
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": "task pickup at p1: task_1"
                }
            }
        },
        "sequence": {
            "type": "mars_agent_physical_robot_msgs.Sequence",
            "value": {
                "length": {
                    "type": "number",
                    "value": 23
                },
                "sequence_number": {
                    "type": "number",
                    "value": 23
                }
            }
        }
    },
    "metadata": {
        "dataType": {
            "type": "dataType",
            "value": {
                "robot_action": {
                    "category": "uint8",
                    "action": "uint8",
                    "description": "string",
                    "attributes": "mars_common_msgs/Tuple[]"
                },
                "action_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "task_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "sequence": {
                    "length": "int32",
                    "sequence_number": "int32"
                }
            }
        }
    }
}
```

#### CancelTask.msg

The cancel task message cancels a whole task for an AGV.

| Type                | Variable  | Description                                                                                                                 |
| ------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id | task_id   | ID of the task which should be canceled. If an Action- or MotionID is additionally given, only this part will be cancelled. |
| mars_common_msgs/Id | action_id | NOT supported at the moment!                                                                                                |
| mars_common_msgs/Id | motion_id | Not supported at the moment!                                                                                                |

##### ROS Message

```ini
# task ID instead of action id because the message deletes the whole task
# the task is a sequence of motions and actions
mars_common_msgs/Id task_id
mars_common_msgs/Id action_id
mars_common_msgs/Id motion_id
```

##### Example of Orion entity

```json
"cancel_order": {
    "type": "mars_agent_physical_robot_msgs.CancelTask",
    "value": {
        "action_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 1
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "task_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 5
                        },
                        {
                            "type": "number",
                            "value": 3
                        },
                        {
                            "type": "number",
                            "value": 6
                        },
                        {
                            "type": "number",
                            "value": 2
                        },
                        {
                            "type": "number",
                            "value": 9
                        },
                        {
                            "type": "number",
                            "value": 7
                        },
                        {
                            "type": "number",
                            "value": 3
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 6
                        },
                        {
                            "type": "number",
                            "value": 8
                        },
                        {
                            "type": "number",
                            "value": 1
                        },
                        {
                            "type": "number",
                            "value": 1
                        },
                        {
                            "type": "number",
                            "value": 7
                        },
                        {
                            "type": "number",
                            "value": 4
                        },
                        {
                            "type": "number",
                            "value": 5
                        },
                        {
                            "type": "number",
                            "value": 3
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "motion_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 1
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        }
    },
    "metadata": {
        "dataType": {
            "type": "dataType",
            "value": {
                "action_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "task_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "motion_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                }
            }
        }
    }
}
```

### Interfaces consumed by TP

#### Motion.msg

The motion message is a combination of the current position of the AGV in the global coordinate system provided by S&P and the current velocity. 

| Type                      | Variable         | Description                                                                                                                                          |
| ------------------------- | ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| geometry_msgs/PoseStamped | current_position | Current position of the AGV. (For more information about the message visit: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) |
| geometry_msgs/Twist       | current_velocity | Current velocity of the AGV. (For more information about the message visit: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)              |

##### ROS Message

```ini
geometry_msgs/PoseStamped current_position
geometry_msgs/Twist current_velocity
```

##### Example of Orion entity

```json
"current_motion": {
    "type": "mars_agent_physical_robot_msgs.Motion",
    "value": {
        "current_position": {
            "type": "geometry_msgs.PoseStamped",
            "value": {
                "header": {
                    "type": "std_msgs.Header",
                    "value": {
                        "stamp": {
                            "type": "Time",
                            "value": {
                                "secs": {
                                    "type": "number",
                                    "value": 126
                                },
                                "nsecs": {
                                    "type": "number",
                                    "value": 800000000
                                }
                            }
                        },
                        "frame_id": {
                            "type": "string",
                            "value": "/map"
                        },
                        "seq": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "pose": {
                    "type": "geometry_msgs.Pose",
                    "value": {
                        "position": {
                            "type": "geometry_msgs.Point",
                            "value": {
                                "y": {
                                    "type": "number",
                                    "value": -5.12
                                },
                                "x": {
                                    "type": "number",
                                    "value": -8.916
                                },
                                "z": {
                                    "type": "number",
                                    "value": 0
                                }
                            }
                        },
                        "orientation": {
                            "type": "geometry_msgs.Quaternion",
                            "value": {
                                "y": {
                                    "type": "number",
                                    "value": 0
                                },
                                "x": {
                                    "type": "number",
                                    "value": 0
                                },
                                "z": {
                                    "type": "number",
                                    "value": 0
                                },
                                "w": {
                                    "type": "number",
                                    "value": 1
                                }
                            }
                        }
                    }
                }
            }
        },
        "current_velocity": {
            "type": "geometry_msgs.Twist",
            "value": {
                "linear": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 0
                        },
                        "z": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "angular": {
                    "type": "geometry_msgs.Vector3",
                    "value": {
                        "y": {
                            "type": "number",
                            "value": 0
                        },
                        "x": {
                            "type": "number",
                            "value": 0
                        },
                        "z": {
                            "type": "number",
                            "value": 0
                        }
                    }
                }
            }
        }
    },
    "metadata": {
        "dataType": {
            "type": "dataType",
            "value": {
                "current_position": {
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "pose": {
                        "position": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        },
                        "orientation": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64",
                            "w": "float64"
                        }
                    }
                },
                "current_velocity": {
                    "linear": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    },
                    "angular": {
                        "y": "float64",
                        "x": "float64",
                        "z": "float64"
                    }
                }
            }
        }
    }
}
```

#### AssignmentStatus.msg

The assignment status gives you an overview which task is currently executed by the AGV and which was the last finished task.

| Type                         | Variable             | Description                                           |
| ---------------------------- | -------------------- | ----------------------------------------------------- |
| mars_common_msgs/Id          | current_task_id      | Id of the current task which is executed.             |
| mars_common_msgs/Id          | current_motion_id    | Id of the current MotionAssignment which is executed. |
| mars_common_msgs/Id          | current_action_id    | Id of the current ActionAssignment which is executed. |
| mars_common_msgs/Id          | last_finished_motion | Id of the last finished MotionAssignment.             |
| mars_common_msgs/Id          | last_finished_action | Id of the last finished ActionAssignment.             |
| geometry_msgs/PolygonStamped | footprint            | Current footprint of the AGV, including load.         |

##### ROS Message

```ini
mars_common_msgs/Id current_task_id
mars_common_msgs/Id current_motion_id
mars_common_msgs/Id current_action_id
mars_common_msgs/Id last_finished_motion
mars_common_msgs/Id last_finished_action
geometry_msgs/PolygonStamped footprint
```

##### Example of Orion entity

Currently not available (message not used at the moment)!

#### RobotAgentDescription.msg

This message describes the AGVs footprint, kinematic and the capabilities like lifting operations. 

| Type                         | Variable          | Description                                                                                                                                                                                                                                        |
| ---------------------------- | ----------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mars_common_msgs/Id          | robot_id          | ID of the robot                                                                                                                                                                                                                                    |
| VehicleType                  | type              | Defines the type of the AGV                                                                                                                                                                                                                        |
| geometry_msgs/PolygonStamped | footprint         | The footprint is the contour of the mobile base. In ROS, it is a two dimensional array of the form [x0, y0],[x1, y1], ..., [xn, yn]]. The origin of the coordinates should be the center of the robot (center of rotation for differential drive). |
| float32                      | min_height        | Minimal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | max_height        | Maximal height of the AGV in meter.                                                                                                                                                                                                                |
| float32                      | payload           | Maximum Payload which can be carried by the AGV in kilogram.                                                                                                                                                                                       |
| float32                      | max_pos_x_vel     | Maximum positive velocity in driving direction in m/s.                                                                                                                                                                                             |
| float32                      | max_neg_x_vel     | Maximum negative speed in reverse direction in m/s.                                                                                                                                                                                                |
| float32                      | max_pos_x_acc     | Maximum positive acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_neg_x_acc     | Maximum negative acceleration in driving direction in m/s².                                                                                                                                                                                        |
| float32                      | max_pos_y_vel     | Maximum positive velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_neg_y_vel     | Maximum negative velocity in y direction for omnidirectional AGV in m/s.                                                                                                                                                                           |
| float32                      | max_pos_y_acc     | Maximum positive acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_neg_y_acc     | Maximum negative acceleration in y direction for omnidirectional AGV in m/s².                                                                                                                                                                      |
| float32                      | max_pos_ang_v     | Maximum positive angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_neg_ang_v     | Maximum negative angular velocity in m/s.                                                                                                                                                                                                          |
| float32                      | max_pos_ang_a     | Maximum positive angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | max_neg_ang_a     | Maximum negative angular acceleration in m/s².                                                                                                                                                                                                     |
| float32                      | velocity_cont     | ???                                                                                                                                                                                                                                                |
| float32                      | min_turning_r     | Turning radius in meter. For differential drives it is zero!                                                                                                                                                                                       |
| float32                      | batt_capacity     | Maximum capacity of the battery in Ah.                                                                                                                                                                                                             |
| float32                      | batt_max_volt     | Maximum voltage of the battery in V.                                                                                                                                                                                                               |
| float32                      | weight            | Weight of the AGV in kg.                                                                                                                                                                                                                           |
| string                       | vendor            | Vendor of the AGV.                                                                                                                                                                                                                                 |
| RobotAction []               | action_capability | A list of Actions which can be performed by the AGV.                                                                                                                                                                                               |

##### ROS Message

```ini
mars_common_msgs/Id robot_id
VehicleType type
geometry_msgs/PolygonStamped footprint
float32 min_height
float32 max_height
float32 payload
float32 max_pos_x_vel
float32 max_neg_x_vel
float32 max_pos_x_acc
float32 max_neg_x_acc
float32 max_pos_y_vel
float32 max_neg_y_vel
float32 max_pos_y_acc
float32 max_neg_y_acc
float32 max_pos_ang_vel
float32 max_neg_ang_vel
float32 max_pos_ang_acc
float32 max_neg_ang_acc
float32 velocity_control_sensitivity
float32 min_turning_radius
float32 batt_capacity
float32 batt_max_voltage
float32 weight
string vendor 
RobotAction[] action_capability

```

##### Example of Orion entity

```json
"robot_description": {
    "type": "mars_agent_physical_robot_msgs.RobotAgentProperties",
    "value": {
        "batt_capacity": {
            "type": "number",
            "value": 1
        },
        "weight": {
            "type": "number",
            "value": 0
        },
        "max_pos_ang_acc": {
            "type": "number",
            "value": 0.5
        },
        "max_neg_x_vel": {
            "type": "number",
            "value": 1.100000024
        },
        "payload": {
            "type": "number",
            "value": 0
        },
        "max_pos_x_vel": {
            "type": "number",
            "value": 1.100000024
        },
        "action_capability": {
            "type": "array",
            "value": [],
            "metadata": {
                "dataType": {
                    "type": "dataType",
                    "value": "mars_agent_physical_robot_msgs/RobotAction[]"
                }
            }
        },
        "robot_id": {
            "type": "mars_common_msgs.Id",
            "value": {
                "uuid": {
                    "type": "array",
                    "value": [
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 0
                        },
                        {
                            "type": "number",
                            "value": 1
                        }
                    ],
                    "metadata": {
                        "dataType": {
                            "type": "dataType",
                            "value": "uint8[16]"
                        }
                    }
                },
                "description": {
                    "type": "string",
                    "value": ""
                }
            }
        },
        "type": {
            "type": "mars_agent_physical_robot_msgs.VehicleType",
            "value": {
                "vehicle_type": {
                    "type": "number",
                    "value": 0
                }
            }
        },
        "max_neg_ang_acc": {
            "type": "number",
            "value": 0.5
        },
        "vendor": {
            "type": "string",
            "value": "default"
        },
        "max_pos_y_vel": {
            "type": "number",
            "value": 1.100000024
        },
        "max_height": {
            "type": "number",
            "value": 0
        },
        "velocity_control_sensitivity": {
            "type": "number",
            "value": 1
        },
        "min_height": {
            "type": "number",
            "value": 0
        },
        "max_pos_y_acc": {
            "type": "number",
            "value": 0.5
        },
        "footprint": {
            "type": "geometry_msgs.PolygonStamped",
            "value": {
                "header": {
                    "type": "std_msgs.Header",
                    "value": {
                        "stamp": {
                            "type": "Time",
                            "value": {
                                "secs": {
                                    "type": "number",
                                    "value": 0
                                },
                                "nsecs": {
                                    "type": "number",
                                    "value": 0
                                }
                            }
                        },
                        "frame_id": {
                            "type": "string",
                            "value": ""
                        },
                        "seq": {
                            "type": "number",
                            "value": 0
                        }
                    }
                },
                "polygon": {
                    "type": "geometry_msgs.Polygon",
                    "value": {
                        "points": {
                            "type": "array",
                            "value": [
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": 0.349999994
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": 0.270000011
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": 0.349999994
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": -0.870000005
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -0.349999994
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": -0.870000005
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                },
                                {
                                    "type": "geometry_msgs.Point32",
                                    "value": {
                                        "y": {
                                            "type": "number",
                                            "value": -0.349999994
                                        },
                                        "x": {
                                            "type": "number",
                                            "value": 0.270000011
                                        },
                                        "z": {
                                            "type": "number",
                                            "value": 0
                                        }
                                    }
                                }
                            ],
                            "metadata": {
                                "dataType": {
                                    "type": "dataType",
                                    "value": "geometry_msgs/Point32[]"
                                }
                            }
                        }
                    }
                }
            }
        },
        "max_pos_ang_vel": {
            "type": "number",
            "value": 0
        },
        "max_neg_y_acc": {
            "type": "number",
            "value": 0.5
        },
        "max_pos_x_acc": {
            "type": "number",
            "value": 0.5
        },
        "max_neg_y_vel": {
            "type": "number",
            "value": 1.100000024
        },
        "max_neg_ang_vel": {
            "type": "number",
            "value": 1.100000024
        },
        "batt_max_voltage": {
            "type": "number",
            "value": 1
        },
        "max_neg_x_acc": {
            "type": "number",
            "value": 0.5
        },
        "min_turning_radius": {
            "type": "number",
            "value": 0
        }
    },
    "metadata": {
        "dataType": {
            "type": "dataType",
            "value": {
                "batt_capacity": "float32",
                "weight": "float32",
                "max_pos_ang_acc": "float32",
                "max_neg_x_vel": "float32",
                "payload": "float32",
                "max_pos_x_vel": "float32",
                "action_capability": "mars_agent_physical_robot_msgs/RobotAction[]",
                "robot_id": {
                    "uuid": "uint8[16]",
                    "description": "string"
                },
                "type": {
                    "vehicle_type": "uint8"
                },
                "max_neg_ang_acc": "float32",
                "vendor": "string",
                "max_pos_y_vel": "float32",
                "max_height": "float32",
                "velocity_control_sensitivity": "float32",
                "min_height": "float32",
                "max_pos_y_acc": "float32",
                "footprint": {
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "polygon": {
                        "points": "geometry_msgs/Point32[]"
                    }
                },
                "max_pos_ang_vel": "float32",
                "max_neg_y_acc": "float32",
                "max_pos_x_acc": "float32",
                "max_neg_y_vel": "float32",
                "max_neg_ang_vel": "float32",
                "batt_max_voltage": "float32",
                "max_neg_x_acc": "float32",
                "min_turning_radius": "float32"
            }
        }
    }
}
```
### Used messages inside TP messages

Used messages are not directly send to another participant in the system. These messages are only a part of other messages.

#### ID.msg
| Type      | Variable    | Description                                                                                                                                                                                                                                               |
| --------- | ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| uint8[16] | uuid        | Universally Unique Identifier (UUID). A UUID is a 128-bit number used to identify information in computer systems.  For generation UUIDs version 4 and 5 is used. For more information visit: https://en.wikipedia.org/wiki/Universally_unique_identifier |
| string    | description | Optional description of the ID. Description can be: Name, what is descriped by the id, etc.                                                                                                                                                               |
##### ROS Message
```ini
# Universally Unique Identifier (UUID)
# A UUID is a 128-bit number used to identify information in computer systems. 
# The term globally unique identifier (GUID) is also used. 
# More information: https://en.wikipedia.org/wiki/Universally_unique_identifier
uint8[16] uuid

# optional description of the id
# description can be:
#   - a name
#   - what is descriped by the id
#   - etc.
string description

```

#### Sequence.msg

| Type  | Variable        | Description                                                |
| ----- | --------------- | ---------------------------------------------------------- |
| int32 | sequence_number | Actual position in the sequence. The sequence starts at 1! |
| int32 | length          | Indicates how long the whole sequence will be.             |

##### ROS Message

```ini
# Message for defining a sequence.

# Actual position in the sequence. The sequence starts at 1! 
int32 sequence_number
# Overall lenght of the sequence
int32 length
```

#### RobotAction.msg

IMPORTANT: The robot action is not finally defined and not used in OPIL v3! 

| Type                     | Variable                          | Description                                                                       |
| ------------------------ | --------------------------------- | --------------------------------------------------------------------------------- |
| uint8                    | CATEGORY_UNDEFINED = 0            | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_NONE = 5                 | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_LOAD = 10                | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_LOAD_START = 11   | CONSTANT VARIABLE. Can be used inside the message for setting the accategorytion. |
| uint8                    | CATEGORY_MANUAL_LOAD_DONE = 12    | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_UNLOAD = 20              | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_UNLOAD_START = 21 | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_MANUAL_UNLOAD_DONE = 22  | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_START_CHARGING = 30      | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | CATEGORY_STOP_CHARGING = 31       | CONSTANT VARIABLE. Can be used inside the message for setting the category.       |
| uint8                    | category                          | Describes the category of the action. See constants!                              |
| uint8                    | action                            | Action which should be executed. Possible actions mus be described by the AGV!    |
| mars_common_msgs/Tuple[] | attributes                        | Additional attributes which are needed to execute the action.                     |
| string                   | description                       | Human readable description of the action.                                         |

##### ROS Message

```ini
# definition of all the possible actionsmars_agent_physical_robot_msgs
uint8 CATEGORY_UNDEFINED = 0
uint8 CATEGORY_NONE = 5
uint8 CATEGORY_LOAD = 10
uint8 CATEGORY_MANUAL_LOAD_START = 11
uint8 CATEGORY_MANUAL_LOAD_DONE = 12
uint8 CATEGORY_UNLOAD = 20
uint8 CATEGORY_MANUAL_UNLOAD_START = 21
uint8 CATEGORY_MANUAL_UNLOAD_DONE = 22
uint8 CATEGORY_START_CHARGING = 30
uint8 CATEGORY_STOP_CHARGING = 31
# ...


# Category of the action which has to be performed
uint8 category
# Defines the robot specific action which has to be performed. 
# The specific actions must be defined by manufacturer. 
uint8 action
mars_common_msgs/Tuple[] attributes
# Optional description of the action. E.g.: unload left
string description
```

#### VehicleType.msg

IMPORTANT: The vehicle type is not finally defined and not used in OPIL v3! 

| Type  | Variable                        | Description                                                                     |
| ----- | ------------------------------- | ------------------------------------------------------------------------------- |
| uint8 | VEHICLE_TYPE_UNKNOWN=0          | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | VEHICLE_TYPE_SUPPLY_VEHICLE=1   | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | VEHICLE_TYPE_ASSEMBLY_VEHICLE=2 | CONSTANT VARIABLE. Can be used inside the message for setting the vehicle type. |
| uint8 | vehicle_type                    | Vehicle type of the AGV. Currently assembly and supply AGV are supported.       |

##### ROS Message

```ini
# supported vehicle_types
uint8 VEHICLE_TYPE_UNKNOWN=0
# TODO:add correct vehicle_types!
uint8 VEHICLE_TYPE_SUPPLY_VEHICLE=1
uint8 VEHICLE_TYPE_ASSEMBLY_VEHICLE=2

# type of the vehicle
uint8 vehicle_type
```

### Complete Entity example
```json
{
    "id": "robot_opil_v2",
    "type": "ROBOT",
    "action_assignment": {
        "type": "mars_agent_physical_robot_msgs.ActionAssignment",
        "value": {
            "robot_action": {
                "type": "mars_agent_physical_robot_msgs.RobotAction",
                "value": {
                    "category": {
                        "type": "number",
                        "value": 10
                    },
                    "action": {
                        "type": "number",
                        "value": 5
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    },
                    "attributes": {
                        "type": "array",
                        "value": [
                            {
                                "type": "mars_common_msgs.Tuple",
                                "value": {
                                    "type": {
                                        "type": "string",
                                        "value": ""
                                    },
                                    "name": {
                                        "type": "string",
                                        "value": ""
                                    },
                                    "value": {
                                        "type": "string",
                                        "value": ""
                                    }
                                }
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "mars_common_msgs/Tuple[]"
                            }
                        }
                    }
                }
            },
            "action_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 31
                            },
                            {
                                "type": "number",
                                "value": 54
                            },
                            {
                                "type": "number",
                                "value": 230
                            },
                            {
                                "type": "number",
                                "value": 47
                            },
                            {
                                "type": "number",
                                "value": 237
                            },
                            {
                                "type": "number",
                                "value": 27
                            },
                            {
                                "type": "number",
                                "value": 86
                            },
                            {
                                "type": "number",
                                "value": 34
                            },
                            {
                                "type": "number",
                                "value": 183
                            },
                            {
                                "type": "number",
                                "value": 123
                            },
                            {
                                "type": "number",
                                "value": 216
                            },
                            {
                                "type": "number",
                                "value": 30
                            },
                            {
                                "type": "number",
                                "value": 6
                            },
                            {
                                "type": "number",
                                "value": 182
                            },
                            {
                                "type": "number",
                                "value": 31
                            },
                            {
                                "type": "number",
                                "value": 11
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    }
                }
            },
            "task_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 126
                            },
                            {
                                "type": "number",
                                "value": 145
                            },
                            {
                                "type": "number",
                                "value": 110
                            },
                            {
                                "type": "number",
                                "value": 222
                            },
                            {
                                "type": "number",
                                "value": 168
                            },
                            {
                                "type": "number",
                                "value": 165
                            },
                            {
                                "type": "number",
                                "value": 90
                            },
                            {
                                "type": "number",
                                "value": 83
                            },
                            {
                                "type": "number",
                                "value": 141
                            },
                            {
                                "type": "number",
                                "value": 113
                            },
                            {
                                "type": "number",
                                "value": 29
                            },
                            {
                                "type": "number",
                                "value": 45
                            },
                            {
                                "type": "number",
                                "value": 22
                            },
                            {
                                "type": "number",
                                "value": 23
                            },
                            {
                                "type": "number",
                                "value": 1
                            },
                            {
                                "type": "number",
                                "value": 132
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": "task pickup at p1: task_1"
                    }
                }
            },
            "sequence": {
                "type": "mars_agent_physical_robot_msgs.Sequence",
                "value": {
                    "length": {
                        "type": "number",
                        "value": 23
                    },
                    "sequence_number": {
                        "type": "number",
                        "value": 23
                    }
                }
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "robot_action": {
                        "category": "uint8",
                        "action": "uint8",
                        "description": "string",
                        "attributes": "mars_common_msgs/Tuple[]"
                    },
                    "action_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "task_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "sequence": {
                        "length": "int32",
                        "sequence_number": "int32"
                    }
                }
            }
        }
    },
    "cancel_order": {
        "type": "mars_agent_physical_robot_msgs.CancelTask",
        "value": {
            "action_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 1
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": "not supported"
                    }
                }
            },
            "task_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 5
                            },
                            {
                                "type": "number",
                                "value": 3
                            },
                            {
                                "type": "number",
                                "value": 6
                            },
                            {
                                "type": "number",
                                "value": 2
                            },
                            {
                                "type": "number",
                                "value": 9
                            },
                            {
                                "type": "number",
                                "value": 7
                            },
                            {
                                "type": "number",
                                "value": 3
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 6
                            },
                            {
                                "type": "number",
                                "value": 8
                            },
                            {
                                "type": "number",
                                "value": 1
                            },
                            {
                                "type": "number",
                                "value": 1
                            },
                            {
                                "type": "number",
                                "value": 7
                            },
                            {
                                "type": "number",
                                "value": 4
                            },
                            {
                                "type": "number",
                                "value": 5
                            },
                            {
                                "type": "number",
                                "value": 3
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": "Cancel task pickup p1"
                    }
                }
            },
            "motion_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 1
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": "not supported"
                    }
                }
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "action_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "task_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "motion_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    }
                }
            }
        }
    },
    "current_motion": {
        "type": "mars_agent_physical_robot_msgs.Motion",
        "value": {
            "current_position": {
                "type": "geometry_msgs.PoseStamped",
                "value": {
                    "header": {
                        "type": "std_msgs.Header",
                        "value": {
                            "stamp": {
                                "type": "Time",
                                "value": {
                                    "secs": {
                                        "type": "number",
                                        "value": 2291
                                    },
                                    "nsecs": {
                                        "type": "number",
                                        "value": 100000000
                                    }
                                }
                            },
                            "frame_id": {
                                "type": "string",
                                "value": "/map"
                            },
                            "seq": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "pose": {
                        "type": "geometry_msgs.Pose",
                        "value": {
                            "position": {
                                "type": "geometry_msgs.Point",
                                "value": {
                                    "y": {
                                        "type": "number",
                                        "value": -3.119975119
                                    },
                                    "x": {
                                        "type": "number",
                                        "value": -0.891966949
                                    },
                                    "z": {
                                        "type": "number",
                                        "value": 0
                                    }
                                }
                            },
                            "orientation": {
                                "type": "geometry_msgs.Quaternion",
                                "value": {
                                    "y": {
                                        "type": "number",
                                        "value": 0
                                    },
                                    "x": {
                                        "type": "number",
                                        "value": 0
                                    },
                                    "z": {
                                        "type": "number",
                                        "value": 0.010969549
                                    },
                                    "w": {
                                        "type": "number",
                                        "value": 0.999939833
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "current_velocity": {
                "type": "geometry_msgs.Twist",
                "value": {
                    "linear": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 0
                            },
                            "z": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "angular": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 0
                            },
                            "z": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    }
                }
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "current_position": {
                        "header": {
                            "stamp": {
                                "secs": "int32",
                                "nsecs": "int32"
                            },
                            "frame_id": "string",
                            "seq": "uint32"
                        },
                        "pose": {
                            "position": {
                                "y": "float64",
                                "x": "float64",
                                "z": "float64"
                            },
                            "orientation": {
                                "y": "float64",
                                "x": "float64",
                                "z": "float64",
                                "w": "float64"
                            }
                        }
                    },
                    "current_velocity": {
                        "linear": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        },
                        "angular": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        }
                    }
                }
            }
        }
    },
    "motion_assignment": {
        "type": "mars_agent_physical_robot_msgs.MotionAssignment",
        "value": {
            "use_orientation": {
                "type": "boolean",
                "value": false
            },
            "is_waypoint": {
                "type": "boolean",
                "value": true
            },
            "task_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 76
                            },
                            {
                                "type": "number",
                                "value": 228
                            },
                            {
                                "type": "number",
                                "value": 77
                            },
                            {
                                "type": "number",
                                "value": 190
                            },
                            {
                                "type": "number",
                                "value": 160
                            },
                            {
                                "type": "number",
                                "value": 186
                            },
                            {
                                "type": "number",
                                "value": 77
                            },
                            {
                                "type": "number",
                                "value": 49
                            },
                            {
                                "type": "number",
                                "value": 145
                            },
                            {
                                "type": "number",
                                "value": 55
                            },
                            {
                                "type": "number",
                                "value": 246
                            },
                            {
                                "type": "number",
                                "value": 239
                            },
                            {
                                "type": "number",
                                "value": 100
                            },
                            {
                                "type": "number",
                                "value": 23
                            },
                            {
                                "type": "number",
                                "value": 251
                            },
                            {
                                "type": "number",
                                "value": 73
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    }
                }
            },
            "sequence": {
                "type": "mars_agent_physical_robot_msgs.Sequence",
                "value": {
                    "length": {
                        "type": "number",
                        "value": 20
                    },
                    "sequence_number": {
                        "type": "number",
                        "value": 20
                    }
                }
            },
            "point_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 211
                            },
                            {
                                "type": "number",
                                "value": 15
                            },
                            {
                                "type": "number",
                                "value": 212
                            },
                            {
                                "type": "number",
                                "value": 98
                            },
                            {
                                "type": "number",
                                "value": 168
                            },
                            {
                                "type": "number",
                                "value": 77
                            },
                            {
                                "type": "number",
                                "value": 89
                            },
                            {
                                "type": "number",
                                "value": 64
                            },
                            {
                                "type": "number",
                                "value": 181
                            },
                            {
                                "type": "number",
                                "value": 51
                            },
                            {
                                "type": "number",
                                "value": 63
                            },
                            {
                                "type": "number",
                                "value": 72
                            },
                            {
                                "type": "number",
                                "value": 110
                            },
                            {
                                "type": "number",
                                "value": 11
                            },
                            {
                                "type": "number",
                                "value": 133
                            },
                            {
                                "type": "number",
                                "value": 97
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    }
                }
            },
            "point": {
                "type": "geometry_msgs.Pose2D",
                "value": {
                    "y": {
                        "type": "number",
                        "value": -3.119999886
                    },
                    "x": {
                        "type": "number",
                        "value": -0.916000009
                    },
                    "theta": {
                        "type": "number",
                        "value": 0
                    }
                }
            },
            "max_velocity": {
                "type": "geometry_msgs.Twist",
                "value": {
                    "linear": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 10
                            },
                            "z": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "angular": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 0
                            },
                            "z": {
                                "type": "number",
                                "value": 1
                            }
                        }
                    }
                }
            },
            "header": {
                "type": "std_msgs.Header",
                "value": {
                    "stamp": {
                        "type": "Time",
                        "value": {
                            "secs": {
                                "type": "number",
                                "value": 1572008882
                            },
                            "nsecs": {
                                "type": "number",
                                "value": 381286634
                            }
                        }
                    },
                    "frame_id": {
                        "type": "string",
                        "value": ""
                    },
                    "seq": {
                        "type": "number",
                        "value": 19
                    }
                }
            },
            "motion_area": {
                "type": "geometry_msgs.PolygonStamped",
                "value": {
                    "header": {
                        "type": "std_msgs.Header",
                        "value": {
                            "stamp": {
                                "type": "Time",
                                "value": {
                                    "secs": {
                                        "type": "number",
                                        "value": 0
                                    },
                                    "nsecs": {
                                        "type": "number",
                                        "value": 0
                                    }
                                }
                            },
                            "frame_id": {
                                "type": "string",
                                "value": ""
                            },
                            "seq": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "polygon": {
                        "type": "geometry_msgs.Polygon",
                        "value": {
                            "points": {
                                "type": "array",
                                "value": [
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -2.170000076
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": 0.034000002
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -2.170000076
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": -1.866000056
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -4.070000172
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": -1.866000056
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -4.070000172
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": 0.034000002
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    }
                                ],
                                "metadata": {
                                    "dataType": {
                                        "type": "dataType",
                                        "value": "geometry_msgs/Point32[]"
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "motion_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 30
                            },
                            {
                                "type": "number",
                                "value": 175
                            },
                            {
                                "type": "number",
                                "value": 251
                            },
                            {
                                "type": "number",
                                "value": 36
                            },
                            {
                                "type": "number",
                                "value": 210
                            },
                            {
                                "type": "number",
                                "value": 224
                            },
                            {
                                "type": "number",
                                "value": 64
                            },
                            {
                                "type": "number",
                                "value": 42
                            },
                            {
                                "type": "number",
                                "value": 173
                            },
                            {
                                "type": "number",
                                "value": 87
                            },
                            {
                                "type": "number",
                                "value": 188
                            },
                            {
                                "type": "number",
                                "value": 44
                            },
                            {
                                "type": "number",
                                "value": 128
                            },
                            {
                                "type": "number",
                                "value": 109
                            },
                            {
                                "type": "number",
                                "value": 196
                            },
                            {
                                "type": "number",
                                "value": 11
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    }
                }
            },
            "max_acceleration": {
                "type": "geometry_msgs.Accel",
                "value": {
                    "linear": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 1
                            },
                            "z": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "angular": {
                        "type": "geometry_msgs.Vector3",
                        "value": {
                            "y": {
                                "type": "number",
                                "value": 0
                            },
                            "x": {
                                "type": "number",
                                "value": 0
                            },
                            "z": {
                                "type": "number",
                                "value": 0.5
                            }
                        }
                    }
                }
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "use_orientation": "bool",
                    "is_waypoint": "bool",
                    "task_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "sequence": {
                        "length": "int32",
                        "sequence_number": "int32"
                    },
                    "point_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "point": {
                        "y": "float64",
                        "x": "float64",
                        "theta": "float64"
                    },
                    "max_velocity": {
                        "linear": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        },
                        "angular": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        }
                    },
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "motion_area": {
                        "header": {
                            "stamp": {
                                "secs": "int32",
                                "nsecs": "int32"
                            },
                            "frame_id": "string",
                            "seq": "uint32"
                        },
                        "polygon": {
                            "points": "geometry_msgs/Point32[]"
                        }
                    },
                    "motion_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "max_acceleration": {
                        "linear": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        },
                        "angular": {
                            "y": "float64",
                            "x": "float64",
                            "z": "float64"
                        }
                    }
                }
            }
        }
    },
    "robot_description": {
        "type": "mars_agent_physical_robot_msgs.RobotAgentProperties",
        "value": {
            "batt_capacity": {
                "type": "number",
                "value": 1
            },
            "weight": {
                "type": "number",
                "value": 0
            },
            "max_pos_ang_acc": {
                "type": "number",
                "value": 0.5
            },
            "max_neg_x_vel": {
                "type": "number",
                "value": 1.100000024
            },
            "payload": {
                "type": "number",
                "value": 0
            },
            "max_pos_x_vel": {
                "type": "number",
                "value": 1.100000024
            },
            "action_capability": {
                "type": "array",
                "value": [],
                "metadata": {
                    "dataType": {
                        "type": "dataType",
                        "value": "mars_agent_physical_robot_msgs/RobotAction[]"
                    }
                }
            },
            "robot_id": {
                "type": "mars_common_msgs.Id",
                "value": {
                    "uuid": {
                        "type": "array",
                        "value": [
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 0
                            },
                            {
                                "type": "number",
                                "value": 1
                            }
                        ],
                        "metadata": {
                            "dataType": {
                                "type": "dataType",
                                "value": "uint8[16]"
                            }
                        }
                    },
                    "description": {
                        "type": "string",
                        "value": ""
                    }
                }
            },
            "type": {
                "type": "mars_agent_physical_robot_msgs.VehicleType",
                "value": {
                    "vehicle_type": {
                        "type": "number",
                        "value": 0
                    }
                }
            },
            "max_neg_ang_acc": {
                "type": "number",
                "value": 0.5
            },
            "vendor": {
                "type": "string",
                "value": "default"
            },
            "max_pos_y_vel": {
                "type": "number",
                "value": 1.100000024
            },
            "max_height": {
                "type": "number",
                "value": 0
            },
            "velocity_control_sensitivity": {
                "type": "number",
                "value": 1
            },
            "min_height": {
                "type": "number",
                "value": 0
            },
            "max_pos_y_acc": {
                "type": "number",
                "value": 0.5
            },
            "footprint": {
                "type": "geometry_msgs.PolygonStamped",
                "value": {
                    "header": {
                        "type": "std_msgs.Header",
                        "value": {
                            "stamp": {
                                "type": "Time",
                                "value": {
                                    "secs": {
                                        "type": "number",
                                        "value": 0
                                    },
                                    "nsecs": {
                                        "type": "number",
                                        "value": 0
                                    }
                                }
                            },
                            "frame_id": {
                                "type": "string",
                                "value": ""
                            },
                            "seq": {
                                "type": "number",
                                "value": 0
                            }
                        }
                    },
                    "polygon": {
                        "type": "geometry_msgs.Polygon",
                        "value": {
                            "points": {
                                "type": "array",
                                "value": [
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": 0.349999994
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": 0.270000011
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": 0.349999994
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": -0.870000005
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -0.349999994
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": -0.870000005
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    },
                                    {
                                        "type": "geometry_msgs.Point32",
                                        "value": {
                                            "y": {
                                                "type": "number",
                                                "value": -0.349999994
                                            },
                                            "x": {
                                                "type": "number",
                                                "value": 0.270000011
                                            },
                                            "z": {
                                                "type": "number",
                                                "value": 0
                                            }
                                        }
                                    }
                                ],
                                "metadata": {
                                    "dataType": {
                                        "type": "dataType",
                                        "value": "geometry_msgs/Point32[]"
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "max_pos_ang_vel": {
                "type": "number",
                "value": 0
            },
            "max_neg_y_acc": {
                "type": "number",
                "value": 0.5
            },
            "max_pos_x_acc": {
                "type": "number",
                "value": 0.5
            },
            "max_neg_y_vel": {
                "type": "number",
                "value": 1.100000024
            },
            "max_neg_ang_vel": {
                "type": "number",
                "value": 1.100000024
            },
            "batt_max_voltage": {
                "type": "number",
                "value": 1
            },
            "max_neg_x_acc": {
                "type": "number",
                "value": 0.5
            },
            "min_turning_radius": {
                "type": "number",
                "value": 0
            }
        },
        "metadata": {
            "dataType": {
                "type": "dataType",
                "value": {
                    "batt_capacity": "float32",
                    "weight": "float32",
                    "max_pos_ang_acc": "float32",
                    "max_neg_x_vel": "float32",
                    "payload": "float32",
                    "max_pos_x_vel": "float32",
                    "action_capability": "mars_agent_physical_robot_msgs/RobotAction[]",
                    "robot_id": {
                        "uuid": "uint8[16]",
                        "description": "string"
                    },
                    "type": {
                        "vehicle_type": "uint8"
                    },
                    "max_neg_ang_acc": "float32",
                    "vendor": "string",
                    "max_pos_y_vel": "float32",
                    "max_height": "float32",
                    "velocity_control_sensitivity": "float32",
                    "min_height": "float32",
                    "max_pos_y_acc": "float32",
                    "footprint": {
                        "header": {
                            "stamp": {
                                "secs": "int32",
                                "nsecs": "int32"
                            },
                            "frame_id": "string",
                            "seq": "uint32"
                        },
                        "polygon": {
                            "points": "geometry_msgs/Point32[]"
                        }
                    },
                    "max_pos_ang_vel": "float32",
                    "max_neg_y_acc": "float32",
                    "max_pos_x_acc": "float32",
                    "max_neg_y_vel": "float32",
                    "max_neg_ang_vel": "float32",
                    "batt_max_voltage": "float32",
                    "max_neg_x_acc": "float32",
                    "min_turning_radius": "float32"
                }
            }
        }
    }
}
```
## TP <-> HMI

The following described entities are exchanged between the HMI and the TP. These entities can be used to define new materialflows and to supervise ongoing materialflows.   

### Interfaces consumed by TP
In order to work as designed, the TaskPlanner requires a Materialflow, which is specified on the TaskLanguage. This entity is explained below.

#### Materialflow

* id : uuid
* type : Materialflow 
* task : Text
* ownerId : Text (Reference to the static UUID of the instance for the HMI)
* active: Boolean (Indidicates, if the Materialflow shall be processed by the TaskSupervisor OR not. Important: ONCE the Hmi shutsdown, the HMI *needs* to set the materialflow to disable. The User needs to enable it manually after an restart of the system. This is required that the system is not doing something unexpected after boot).

Example:

```JSON
{
	"id": "Materialflow1",
	"type": "Materialflow",
	"specification": {
		"value": "template+Position%0A++++position%0A++++type%0Aend%0A%0Atemplate+Sensor%0A++++sensorId%0A++++type%0Aend%0A%0A%23%23%23%23%0A%0ASensor+opticalSensor%0A++++sensorId+%3D+%22optical_sensor1%22%0A++++type+%3D+%22Boolean%22%0Aend%0A%0APosition+moldingPallet%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22Tag10_11%22%0Aend%0A%0APosition+warehouse_pos1%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22Tag12%22%0Aend%0A%0A%23%23%23%23+%0Atask+Transport_Start%0A++++Transport%0A++++from+moldingPallet%0A++++to+warehouse_pos1%0A++++TriggeredBy+opticalSensor.value+%3D%3D+True%0Aend%0A",
		"type": "Text"
	},
	"ownerId": {
		"type": "Text",
		"value": "reviewers hmi"
	},
	"active": {
		"type": "boolean",
		"value": true
	}
}
```

### Interfaces produced by TP
 We recommend to read the TaskPlanner/TaskSupervisor introduction to understand our concept of the TaskSupervisor.

#### TransportOrderUpdate 
Once, a Transportion by an AGV starts, the TaskPlanner will create a TransportOrderUpdate. 

* id : uuid of the running instance
* type : TransportOrderUpdate
* pickupFrom : string (name of the pickup location)
* deliverTo : string (name of the delivery location)
* name : string (name of this task)
* refMaterialflowUpdateId : id (where has been this transportation defined)
* refOwnerId : id (who has this materialflow/transportation defined)
* taskInfo : int (Idle = 0, WaitForStartTrigger = 1, MovingToPickupDestination = 2, WaitForLoading = 3, MovingToDeliveryDestination = 4, WaitForUnloading = 5)
* updateTime : string (last update of this entity)
* startTime : string (when it has been started)

    
```json
{
    "id": "cb01834e-e34d-42e9-95e5-4b9f461065b3",
    "type": "TransportOrderUpdate",
    "deliverTo": {
      "type": "string",
      "value": "Tag10_11",
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "unicode"
        }
      }
    },
    "name": {
      "type": "string",
      "value": "SupplyTaskFromStorage1ToWorkstation1",
      "metadata": {}
    },
    "pickupFrom": {
      "type": "string",
      "value": "Tag12",
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "unicode"
        }
      }
    },
    "refMaterialflowUpdateId": {
      "type": "string",
      "value": "e01b62e6-efee-11e9-b956-e4b97aae3491",
      "metadata": {}
    },
    "refOwnerId": {
      "type": "string",
      "value": "reviewers hmi",
      "metadata": {}
    },
    "startTime": {
      "type": "string",
      "value": "2019-10-16 10:28:05.288958",
      "metadata": {}
    },
    "state": {
      "type": "string",
      "value": "init",
      "metadata": {}
    },
    "taskInfo": {
      "type": "number",
      "value": 0,
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "int"
        }
      }
    },
    "updateTime": {
      "type": "string",
      "value": "2019-10-16 10:28:05.310872",
      "metadata": {}
    }
  }

```



#### MaterialflowSpecificationState  
This entity provides information about the Materialflow and the processed TaskLanguage.

* id : uuid
* type : MaterialflowSpecificationState
* message : String
* refId: String (reference to the Materialflow Entity)
* state: number (0 == ok, -1 ==  error, >0 == ok with some additional information (tbd))

Example:

```Json
{
	"id": "MaterialflowSpecificationState3fcd7747-7428-44cb-bbfb-2d04509addac",
	"type": "MaterialflowSpecificationState",
	"message": {
		"type": "string",
		"value": "Success",
		"metadata": {}
	},
	"refId": {
		"type": "string",
		"value": "Materialflow1",
		"metadata": {
			"python": {
				"type": "dataType",
				"value": "unicode"
			}
		}
	},
	"state": {
		"type": "number",
		"value": 0,
		"metadata": {
			"python": {
				"type": "dataType",
				"value": "int"
			}
		}
	}
}
```