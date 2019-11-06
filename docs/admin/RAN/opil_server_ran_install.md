# How to start the TP docker container
This docker container starts the RAN which receives motion or action tasks from the TP.

To start a docker container prepare a docker-compose.yml following this example for the local machine:

```yml
services: 
  opil.mod.sw.tp.ts:
    image: "l4ms/opil.sw.tp.ts"
    depends_on: 
      - opil.mod.sw.tp.mtp
    environment: 
      - PYTHONUNBUFFERED=1
      - "ROS_MASTER_URI=http://opil.mod.sw.tp.mtp:11311"
    volumes:
      - ./ts_firos_config.ini:/catkin_ws/src/taskplanner/firos_config.ini 
    ports: 
      - "2906:2906" 

  opil.mod.sw.tp.mtp: 
    image: "l4ms/opil.sw.tp.mtp:latest"
    environment: 
      - "ROS_MASTER_URI=http://localhost:11311"
      - DISPLAY=$DISPLAY
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./mod_sw_tp.launch:/catkin_ws/src/mod_sw_tp/launch/mod_sw_tp.launch 
      - ./firos_robots.json:/catkin_ws/src/firos/config/robots.json
      - ./firos_whitelist.json:/catkin_ws/src/firos/config/whitelist.json
      - ./firos_config.json:/catkin_ws/src/firos/config/config.json
    ports: 
      - "11311:11311"
      - "39001:39001"
            
version: "3.5"
```
Before you can start the docker container, some configuration files must be created. Create this files beside your **docker-compose.yml**
```
- firos_config.json
- firos_robots.json
- firos_whitelist.json
- mod_iot_ran.launch
- simulation.launch
- stage.world

```
```
touch firos_config.json firos_robots.json firos_whitelist.json mod_iot_ran.launch simulation.launch stage.world
```
A detailed description of this files is given in the subsequent sections.

To start the container from the folder where you put your docker-compose.yml file execute the following commands:

```
xhost local:root
docker-compose up
```

After you started it a window opens where you can see the configured ran in a ROS rviz window. 

In windows OS you need to open command prompt and type docker-compose up from the folder where you saved docker-compose.yml. The display in windows OS does not work, so you will not be able to see visualizations of topic exchange in rviz, but you can see subscriptions and entities in a web browser.

# Configuration of Firos

The communication with TP through orion is realized via firos. For subscribing to the right topics on ROS site and subscribing to the right entities on orion site, firos needs to be configured. Therefore three different configurations are needed. Create the following empty files beside the above created ***docker-compose.yml***.

```
- firos_config.json
- firos_robots.json
- firos_whitelist.json
```

A detailed description of firos is given here: https://firos.readthedocs.io/en/latest/index.html

## firos_config.json

The ***config.json*** describes the local and remote connection to the orion context broker. For a detailed description follow these instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html

Copy this content to ***firos_config.json*** (preconfigured for RAN v3.0.0-alpha):
```json
{
  "environment": "docker",
  "docker": {
    "server": {
      "port": 10101
    },
    "contextbroker": {
      "address": "<orion address>",
      "port": 1026,
      "subscription": {
        "throttling": 0,
        "subscription_length": 300,
        "subscription_refresh_delay": 0.5
      }
    },
    "endpoint": {
      "address": "<local network address>",
      "port": 39000
    },
    "log_level": "INFO"
  }
}
```

## firos_robots.json

In version 3.0.0-alpha only one robot is preconfigured. The data of the robot is published to following topics:

```
- /robot_opil_v2/current_motion
- /robot_opil_v2/robot_description
```

The following topics are subscribed from RAN:

```
- /robot_opil_v2/motion_assignment
- /robot_opil_v2/action_assignment
- /robot_opil_v2/cancel_order
```

If you add more robots to the system you have to add the topics with the namespace here. For a detailed description of how to setup the robots.json follow this instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html#robotsjson

The following naming convention should be used:
```
/opil/iot/ran_<uuid>
E.g.: /opil/iot/ran_fb3d75a1a82550ffb3e6e4b2bcae482e (uuid v5 for "robot_0")
```

Copy this content to ***firos_robots.json*** (preconfigured for RAN and TP v3.0.0-alpha):
```json
{
    "robot_opil_v2": {
        "topics": {
            "current_motion": {
                "msg": "mars_agent_physical_robot_msgs.msg.Motion",
                "type": "subscriber"
            },
            "robot_description": {
                "msg": "mars_agent_physical_robot_msgs.msg.RobotAgentProperties",
                "type": "subscriber"
            },
            "cancel_order": {
                "msg": "mars_agent_physical_robot_msgs.msg.CancelTask",
                "type": "publisher"
            },
            "motion_assignment": {
                "msg": "mars_agent_physical_robot_msgs.msg.MotionAssignment",
                "type": "publisher"
            },
            "action_assignment": {
                "msg": "mars_agent_physical_robot_msgs.msg.ActionAssignment",
                "type": "publisher"
            }
        }
    }
}
```

## firos_whitelist.json

The ***whitelist.json*** is needed by firos to subscribe to the topics described in the robots.json. All topics from the robots.json must be listed here. For a more detailed description follow this instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html#whitelistjson

Copy this content to ***firos_whitelist.json*** (preconfigured for RAN and TP v3.0.0-alpha):
```json
{
    "robot_opil_v2": {
        "publisher": [
            "cancel_order",
            "motion_assignment",
            "action_assignment"
        ],
        "subscriber": [
            "current_motion",
            "robot_description"
        ]
    }
}
```

# Configuration RAN

The RAN consists of some different configuration files. The **mod_iot_ran.launch** file is for the RAN configuration, the other files (**simulation.launch**, **stage.world**) are for a build in ros simulation besides the Visual Components simulation. The here include simulation simulates the real hardware for testing the ran.

Copy the following configuration to the mod_iot_ran.launch file. A detailed description of the file can be found below.

```xml
<launch>
  <arg name="robot_1_name" default="robot_1" />

  <arg name="initial_pose_robot_1_x" default="-8.916"/>
  <arg name="initial_pose_robot_1_y" default="-5.12"/>
  <arg name="initial_pose_robot_1_a" default="0.0"/>

  <arg name="robot_1_description" default="robot_description_default"/> <!--  DON'T CHANGE THIS LINE! -->

  <arg name="robot_1_id" default="00000000-0000-0000-0000-000000000001"/>
  
  <!-- ****** Firos ***** -->
  <node name="firos" pkg="firos" type="core.py"/> <!-- DON'T CHANGE THIS LINE! -->
  
  <!-- RVIZ -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find mars_simulation)/rviz/opil_finnland.rviz" />

  <!--  ****** Stage simulation *****  -->
  <include file="$(find mars_simulation)/launch/opil_finnland_simulation.launch"/>

  <!-- <group ns="/opil/iot"> -->

    <!-- BEGIN ROBOT 0 -->
    <group ns="robot_opil_v2">
      <param name="tf_prefix" value="robot_1" />

      <include file="$(find mars_simulation_ctv_agent)/launch/mars_simulation_ctv_agent.launch">
        <arg name="robot_id" value="$(arg robot_1_id)"/>
        <arg name="robot_name" value="$(arg robot_1_name)"/>
        <arg name="initial_pose_x" value="$(arg initial_pose_robot_1_x)"/>
        <arg name="initial_pose_y" value="$(arg initial_pose_robot_1_y)"/>
        <arg name="initial_pose_a" value="$(arg initial_pose_robot_1_a)"/>
        <arg name="robot_description" value="$(arg robot_1_description)"/>
        <arg name="scan_topic" value="/robot_1/base_scan" />
      </include>

      <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 map robot_1/odom" />

      <node name="fake_localization" pkg="fake_localization" type="fake_localization">
        <param name="odom_frame_id" value="robot_1/odom"/>
        <param name="base_frame_id" value="robot_1/base_link"/>
      </node>

      <!--  ***************** Robot Model *****************  -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mars_simulation)/urdf/ctv_1.xacro'" />
      <node pkg="robot_state_publisher" type="state_publisher" name="robot_1_state_publisher" />
    </group>
    <!-- END ROBOT 0 -->
  <!-- </group> -->

</launch>
```

The parameter **robot_1_name** sets the name of the robot inside the RAN ROS node.
```xml
  <arg name="robot_1_name" default="robot_1" />
```

The parameter **initial_pose_robot_1_x** sets the current x-position of the robot in map coordinates.
```xml
  <arg name="initial_pose_robot_1_x" default="-8.916"/>
```

The parameter **initial_pose_robot_1_y** sets the current y-position of the robot in map coordinates.
```xml
  <arg name="initial_pose_robot_1_y" default="-5.12"/>
```

The parameter **initial_pose_robot_1_a** sets the current orientation of the robot.
```xml
  <arg name="initial_pose_robot_1_a" default="0.0"/>
```

The parameter **robot_1_description** sets the robot parameter for the simulation and robot description. DON'T CHANGE THIS LINE!
```xml
  <arg name="robot_1_description" default="robot_description_default"/>
```

The parameter **robot_1_id** sets the ID of the robot. The ID must be unique and can be random generated or from the robot name. The ID follows the UUID standard (https://en.wikipedia.org/wiki/Universally_unique_identifier). For generating a random UUID the v4 standard must be used, for generating a UUID from a given name UUID standard v5 must be used.
```xml
  <arg name="robot_1_id" default="00000000-0000-0000-0000-000000000001"/>
```

This line is necessary to start firos and communicate with orion. DON'T CHANGE THIS LINE!
```xml
  <!-- ****** Firos ***** -->
  <node name="firos" pkg="firos" type="core.py"/>
```

This line starts rviz which displays the current position of the robot. If no visualization is favoured, comment this line out.
```xml  
  <!-- RVIZ -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find mars_simulation)/rviz/opil_finnland.rviz" />
```

This line starts the build in simulation.
```xml
  <!--  ****** Stage simulation *****  -->
  <include file="$(find mars_simulation)/launch/opil_finnland_simulation.launch"/>
```
 Sets the namespace for the runs node(s). (In v3.0.0-alpha no namespace is used)
```xml
  <!-- <group ns="/opil/iot"> -->
  .. some content ...
  <!-- </group> -->
```

Starts the RAN node for one robot. Each robot needs an individual RAN. All needed parameter are configured via the arguments above (**arg**).
```xml
  <include file="$(find mars_simulation_ctv_agent)/launch/mars_simulation_ctv_agent.launch">
  .. allot of parameter ...
  </include>
```

Starts a transformation and a localization for the robot in the simulation. Dont't change this lines 
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 map robot_1/odom" />

      <node name="fake_localization" pkg="fake_localization" type="fake_localization">
        <param name="odom_frame_id" value="robot_1/odom"/>
        <param name="base_frame_id" value="robot_1/base_link"/>
      </node>

      <!--  ***************** Robot Model *****************  -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mars_simulation)/urdf/ctv_1.xacro'" />
      <node pkg="robot_state_publisher" type="state_publisher" name="robot_1_state_publisher" />
```

## Simulation configuration
For starting the simulation, two config files are needed. Copy the content of the documentation below to your local files. Don't change the content of the files!

### Configuration of stage.world

```
include "multishuttle.inc"

resolution 0.05 # set the resolution of the underlying raytrace model in meters
interval_sim 100 # simulation timestep in milliseconds
#interval_real 100

window
( 
  size [ 800 800 ] 
  scale 21.642 
)

define map model
(
  color "gray30"
  gui_nose 0
  gui_grid 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)

map
(
  name "opil_finnland"
  size [26.891  15.126 1.000]
  pose [0.000 0.000 0.000 0.000]
  bitmap "opil_finnland.png"
)

# robots added here automatically
multishuttle( pose [ -200 -200 0.000 0.000 ] name "robot_0" color "white") # don't change this robot!
multishuttle( pose [ -8.916 -5.12 0.000 0.000 ] name "robot_0" color "yellow")

```

### Configuration of simulation.launch

```xml
<launch>

  <arg name="robot_1_ns" default="00000000000000000000000000000001"/>

  <master auto="start"/>
  <param name="/use_sim_time" value="true"/>

  <!--  ****** Maps *****  -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find mars_simulation_data)/world/opil_finnland.yaml">
    <param name="frame_id" value="/map"/>
  </node>

  <node pkg="stage_ros" type="stageros" name="stage_ros" args="$(find mars_simulation_data)/world/opil_finnland.world" respawn="false">
    <param name="base_watchdog_timeout" value="0.2"/>

    <!-- Remaps -->
    <remap from="robot_1/cmd_vel" to="/robot_opil_v2/cmd_vel" />

  </node>

</launch>

```