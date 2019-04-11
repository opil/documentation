# RAN interfaces 
As said before, one of the tasks of the RAN is to work as a bridge between the various components of the OPIL architecture. 

## Channels, Topics and messages

   ![communication architecture](../imgs/comArc.jpg)
   
   ![channels details](../imgs/archMex.png)
  
## Messages Structure

The entities exchanged with the RAN and various components are defined as ROS messages. The messages are MotionAssignment, ActionAsignment, CancelTask, RANState, RobotDescription, Twist, RobotState, RobotDescriptionAGV.

### Assignments

### Motion Assignment

 The MotionAssignment is exchanged on the motion_channel topic and contains attributes that influence the movement of the robot from the starting point (the actual position) to a destination (waypoint or goal). Each time one MotionAssignment is received, it is saved into a list.
 With the same message it is also possible to update an existing motion assignment. 

| Attribute name | ROS type | description |
| --- | --- | --- |
| header	| std_msgs/Header | |	
| robot_id | Id | |	
| point_id | Id | |	
| task_id | Id | |	
| point | geometry_msgs/Pose2D | the position to reach (x,y,theta) |
| is_waypoint | bool | true if it is a waypoint, false if it is the final goal |
| use_orientation | bool | true if the robot needs to take into account the theta of the Pose2D, in case it is false the orientation is undefined |
| max_velocity | geometry_msgs/Twist | limit on velocity, uses only linear.x component of the whole structure |
| max_acceleration | geometry_msgs/Accel | limit on acceleration (same as before) |
| motion_area | geometry_msgs/PolygonStamped | definition of an area of possible movement – with reference |
| sequence | Sequence | in which position into a sequence the motion is |


### ActionAssignment

 The ActionAssignment is exchanged on the action_channel and defines what the robot is expected to do when it reaches the goal. The Action defines what the robot needs to do when it has reached the final motion goal. Each received Action assignment is saved into a list. With the same message, it is also possible to update an existing Action Assignment


| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| robot_id | Id | |
| action_id | Id | |
| task_id | Id | |
| sequence | Sequence | in which position into a sequence the action is |
| action | uint8 (enumeration) | the type of action to be performed once the goal is reached |

Both of these entities are “produced” by the Task Planner (TP) and are then sent via FIWARE – Orion Context Broker (F-OCB) to the Robot Agent Node (RAN). The RAN then uses the data contained in the MotionAssignment to manage the navigation of the robot, while the ActionAssignment is simply sent as-it-is to the robot base to be accomplished. It will be on the robot manufacturer to manage the implementation of the action.
 It is assumed that a task is a combination of points (MotionAssignments) and actions (ActionAssignments). Accomplishing a task means moving to a target goal through many waypoints and, upon reaching destination, performing a succession of actions. The link between points and actions is set using the task_id field. Actions and motions with the same task_id belong to the same task and therefore create a sequence. Notice that in a task there cannot be an action followed by a movement. In order to do so, the task has to be splitted in smaller sub-task.

### ActionDefinition 

The same data structure of ActionAssignment is also exchanged on the action_channel_AGV, but with the name of Action Definition. This structure is just a placeholder and will be then redefined with the help of other members of OPIL.

| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| robot_id | Id | |
| action_id | Id | |
| task_id | Id | |
| sequence | Sequence | in which position into a sequence the action is |
| action | uint8 (enumeration) | the type of action to be performed once the goal is reached |
### CancelTask 

In order to remove a task, an action or a motion from the internal queue, the CancelTask message (exchanged on the task_management_channel) can be used, specifying the ID of the task (task_id) or the action (task_id and action_id, with motion_id = 0) or the motion (task_id and motion_id, with action_id = 0) to be cancelled.

| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| robot_id | Id | |
| task_id | Id | |
| action_id | Id | the action to cancel – 0 if sent for a motion |
| motion_id | Id | the motion to cancel – 0 if sent for an action |

### RANState 

The RAN periodically receives updates from the robots. These messages concern the robot status, the battery level, the current position and so on. They are then converted into the RANState message and forwarded through the status_channel.

| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| robot_id | Id | |
| current_task_id | Id | |
| current_motion_id | Id | Those two fields cannot be both valid at the same time. One has to be NULL |
| current_action_id | Id | Those two fields cannot be both valid at the same time. One has to be NULL |
| agv_msg | RobotStatus | data coming from the AGV |
| current_position | geometry_msgs/Pose2D | |
| current_velocity | geometry_msgs/Twist | |
| footprint | geometry_msgs/PolygonStamped | Current state robot representation |
| ran_status | uint8 (enum) | The RAN status; 0: waiting, 1: moving, 2: acting |
| assignment_queue | string | list of saved assignment |

The assignment_queue attribute contains the list of active (not cancelled) assignments. The structure of the string for each assignment is as follows:

*- Task ID: # - Motion ID: # - Action ID: #\n

The # placed here represent the value for the IDs of the assignment.
 As an assignment can be only of one type (Action or Motion), any assignment cannot have both Motion ID and Action ID different from zero.
 An example of this message can be:

*- Task ID: 1 - Motion ID: 1 - Action ID: 0\n ------ 1st motion assignment

*- Task ID: 1 - Motion ID: 2 - Action ID: 0\n ------ 2nd motion assignment

*- Task ID: 1 - Motion ID: 0 - Action ID: 1\n ------ 1st action assignment

### RobotDescription

 While booting, the robot publishes information about its mechanical parameters. These parameters are collected by the RAN, converted into the RobotDescription message and forwarded through the description_channel.
| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| robot_id | Id | |
| agv_msg | RobotDescriptionAGV | data coming from the AGV |

### Twist

 This is a basic ROS message, that is used to represent velocities over three axes. In cmd_vel topic, it commands the movement of the robot. Its structure is described [here](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html)

### RobotState

 The RAN periodically receives updates from the robots on the status_channel_AGV topic. These messages concern the robot status, the battery level, the current position and so on. They are received by the RAN as a RobotState message, then converted into the RANState message and forwarded through the status_channel.

| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| vehicle_id | Id | “hardware” name of the robot |
| robot_status | industrial_msgs/RobotStatus | [info](http://docs.ros.org/jade/api/industrial_msgs/html/msg/RobotStatus.html) |
| battery_status | sensors_msgs/BatteryStatus | [info](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/BatteryState.html) |
| current_position | geometry_msgs/Pose2D | |
| current_velocity | geometry_msgs/Twist | |
| footprint | geometry_msgs/PolygonStamped | Current state robot representation |

It is possible to omit some of the attributes of the message, in case they are not available for the machine.

### RobotDescriptionAGV

 While booting, the robot publishes information about its mechanical parameters. These parameters are collected by the RAN as a RobotDescriptionAGV message on the description_channel_AGV, then converted into the RobotDescription message and forwarded through the description_channel.

| Attribute name | ROS type | description |
| --- | --- | --- |
| header | std_msgs/Header | |
| vehicle_id | Id | “hardware” name of the robot |
|left_size | float32 | geometrical dimensions of the robot [cm] |
| right_size | float32 | geometrical dimensions of the robot [cm] |
| front_size | float32 | geometrical dimensions of the robot [cm] |
| rear_size | float32 | geometrical dimensions of the robot [cm] |
| min_height | float32 | geometrical dimensions of the robot [cm] |
| max_height | float32 | geometrical dimensions of the robot [cm] |
| payload | float32 | maximum load capacity [kg] |
| max_pos_x_vel | float32 | kinetical limitations |
| max_neg_x_vel | float32 | kinetical limitations |
| max_pos_x_acc | float32 | kinetical limitations |
| max_neg_x_acc | float32 | kinetical limitations |
| max_pos_y_vel | float32 | kinetical limitations |
| max_neg_y_vel | float32 | kinetical limitations |
| max_pos_y_acc | float32 | kinetical limitations |
| max_neg_y_acc | float32 | kinetical limitations |
| max_pos_ang_vel | float32 | kinetical limitations |
| max_neg_ang_vel | float32 | kinetical limitations |
| velocity_control_sensitivity | float32 | |
| min_turning_radius | float32 | |
| batt_capacity | float32 | |
| vehicle_type | string | |
| vendor | string | 

It is possible to omit some of the attributes of the message, in case they are not available for the machine.

### ErrorAGV 

The ErrorAGV message circulates in the error_channel_AGV and it is sent by the AGV to the RAN when an error occurred. Its structure is to be refined through further call with other OPIL members

| Attribute name | ROS type | description |
| --- | --- | --- |
| vehicle_id | Id | identifier of the AGV |
| error_AGV | uint8 | code describing the error (to be defined) |
| description | string | |


### ErrorRAN

 The ErrorRAN message circulates in the error_channel and it is sent by the RAN to the OCB when an error occurred. It can be a RAN error or it might forward an AGV error.  Its structure is to be refined through further call with other OPIL members

| Attribute name | ROS type | description |
| --- | --- | --- |
| vehicle_id | Id | identifier of the AGV |
| error_AGV | uint8 | code describing the error (to be defined) |
| description | string | |
| agv_error | ErrorAGV | if it is a forward, otherwise do not use this attribute |


### Useful Data Structure

Id message defines the objects IDs

| Attribute name | ROS type | description |
| --- | --- | --- |
| id | uint32 | numeric ID |
| description | string | alphanumeric ID |

The Sequence message defines the sequence of assignments with a length and a position of the current assignment

| Attribute name | ROS type | description |
| --- | --- | --- |
| sequence_number | int32 | actual position in the sequence |
| length | int32 | sequence length |

The AssignmentId message collects the numeric IDs of an assignment. Action assignments have motion_id = 0, Motion assignments have action_id = 0

| Attribute name | ROS type | description |
| --- | --- | --- |
| task_id | uint32 | numeric task ID |
| motion_id | uint32 | numeric motion ID |
| action_id | uint32 | numeric action ID |


