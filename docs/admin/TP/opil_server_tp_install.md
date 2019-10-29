# Introduction
 
Welcome to Module's Installation & Administration Guide! 

Any feedback on this document is highly welcome, including bug reports, typos or information you think should be included but is not. Please send the feedback through email to: module@l4ms.eu. Thank you in advance.

## How to start the TP docker container
This docker container starts the TP module, which receives the created graph of SP via Orion Context Broker.

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
      - ./ts_fiware_config.ini:/catkin_ws/src/taskplanner/fiware_config.ini 
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
- mod_sw_tp.launch
- ts_fiware_config.ini
```
```
touch firos_config.json firos_robots.json firos_whitelist.json mod_sw_tp.launch ts_fiware_config.ini
```
A detailed description of this files is given in the subsequent sections.

To start the container from the folder where you put your docker-compose.yml file execute the following commands:

```
xhost local:root
docker-compose up
```

After you started it a window opens where you can see the graph. It takes some time until the whole graph is ready. If no edges or vertices are longer added to the graph TP is ready to use.

In windows OS you need to open command prompt and type docker-compose up from the folder where you saved docker-compose.yml. The display in windows OS does not work, so you will not be able to see visualizations of topic exchange in rviz, but you can see subscriptions and entities in a web browser.

### Configuration of Firos

The communication with RAN and SP through orion is realized via firos. For subscribing to the right topics on ROS site and subscribing to the right entities on orion site, firos needs to be configured. Therefore three different configurations are needed. Create the following empty files beside the above created ***docker-compose.yml***.

```
- firos_config.json
- firos_robots.json
- firos_whitelist.json
```

A detailed description of firos is given here: https://firos.readthedocs.io/en/latest/index.html

#### firos_config.json

The ***config.json*** describes the local and remote connection to the orion context broker. For a detailed description follow these instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html

Copy this content to ***firos_config.json*** (preconfigured for RAN and TP v3.0.0-alpha):
```json
{
  "environment": "docker",

  "docker": {
    "server": {
        "port"      : 10102
    },
    "contextbroker": {
        "address"   : "<orion address>",
        "port"      : 1026,
        "subscription": {
          "throttling": 0,
          "subscription_length": 300,
          "subscription_refresh_delay": 0.5
        }
    },
    "endpoint": {
      "address": "<local network address>",
      "port": 39001
    },
    "log_level": "INFO"
  }
}
```

#### firos_robots.json

The ***robots.json*** subscribe to the topic of /map/graph which is provided by SP to generate the graph for the routing. In version 3.0.0-alpha only one robot is preconfigured. The data for the robot is published to following topics:

```
- /robot_opil_v2/motion_assignment
- /robot_opil_v2/action_assignment
- /robot_opil_v2/cancel_order
```

The following topics are subscribed from RAN:

```
- /robot_opil_v2/current_motion
- /robot_opil_v2/robot_description
```

If you add more robots to the system you have to add the topics with the namespace here. The namespace is given by the RAN and has to be used. For a detailed description of how to setup the robots.json follow this instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html#robotsjson

The following naming convention should be used:
```
/opil/iot/ran_<uuid>
E.g.: /opil/iot/ran_fb3d75a1a82550ffb3e6e4b2bcae482e (uuid v5 for "robot_0")
```

Copy this content to ***firos_robots.json*** (preconfigured for RAN and TP v3.0.0-alpha):
```json
{
    "map": {
        "topics": {
            "graph": {
                "msg": "maptogridmap.msg.Graph",
                "type": "publisher"
            }
        }
    },
    "robot_opil_v2": {
        "topics": {
            "current_motion": {
                "msg": "mars_agent_physical_robot_msgs.msg.Motion",
                "type": "publisher"
            },
            "robot_description": {
                "msg": "mars_agent_physical_robot_msgs.msg.RobotAgentProperties",
                "type": "publisher"
            },
            "cancel_order": {
                "msg": "mars_agent_physical_robot_msgs.msg.CancelTask",
                "type": "subscriber"
            },
            "motion_assignment": {
                "msg": "mars_agent_physical_robot_msgs.msg.MotionAssignment",
                "type": "subscriber"
            },
            "action_assignment": {
                "msg": "mars_agent_physical_robot_msgs.msg.ActionAssignment",
                "type": "subscriber"
            }
        }
    }
}
```

#### firos_whitelist.json

The ***whitelist.json*** is needed by firos to subscribe to the topics described in the robots.json. All topics from the robots.json must be listed here. For a more detailed description follow this instructions: https://firos.readthedocs.io/en/latest/install/configuration-files.html#whitelistjson

Copy this content to ***firos_whitelist.json*** (preconfigured for RAN and TP v3.0.0-alpha):
```json
{
    "map": {
        "publisher": [
            "graph"
        ],
        "subscriber": []
    },
    "robot_opil_v2": {
        "publisher": [
            "current_motion",
            "robot_description"
        ],
        "subscriber": [
            "cancel_order",
            "motion_assignment",
            "action_assignment"
        ]
    }
}
```

### Configuration of MTP

The MTP consist of three different modules: Topology, Router and Logical Agents. The topology is started by a module called *mars_topology_launcher* which listens to the topic /map/graph which is provided by SP. After a map is received, the topology is started automatically by the module. Subsequently the TP is ready to use.

The whole configuration file is listed below (mod_sw_tp.launch, preconfigured for RAN and TP v3.0.0-alpha):

```xml
<launch>

  <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 world map" />

  <!--  ****** Topology *****  -->
  <include file="$(find mars_topology_launcher)/launch/mars_topology_launcher_generic.launch">
    <arg name="log_level" value="info" />
    <arg name="topo_file_type" value="opil_sp" />
    <arg name="mars_vertex_footprint_radius" value="0.95" />
  </include>

  <!-- ****** Router ***** -->
  <include file="$(find mars_routing_base)/launch/mars_routing_base.launch" />

  <!-- ****** Logical Agent (robot_0) ***** -->
  <include file="$(find mars_agent_logical_agv)/launch/mars_agent_logical_agv.launch">
    <arg name="physical_robot_namespace" value=""/>
    <arg name="robot_name" value="robot_opil_v2" />
    <arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
    <arg name="physical_agent_description" value="robot_0" />
    <arg name="current_topology_entity_id" value="0a8b9081-d84c-5660-909c-134d55bf4966" />
    <!-- p0 -->
    <arg name="node_name" value="ran_00000000000000000000000000000001" />
  </include>

  <!-- ****** Firos ***** -->
  <node name="firos" pkg="firos" type="core.py"/>

  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find mod_sw_tp)/rviz/config.rviz" />

</launch>
```

Following you find a more detailed description of the launch file and the parameter.

#### Topology

Following you can see the configuration for the topology launcher module:

```xml
  <!--  ****** Topology *****  -->
  <include file="$(find mars_topology_launcher)/launch/mars_topology_launcher_generic.launch">
    <arg name="log_level" value="info" /> <!--log levels: debug, info, warn, error -->
    <arg name="topo_file_type" value="opil_sp" /> <!-- don't change this line -->
    <arg name="mars_vertex_footprint_radius" value="0.95" /> <!-- IMPORTANT: This value must be smaller (mars_vertex_footprint_radius < cell_size) then the cell_size of SP!-->
  </include>
```

The log level for starting the topology can be set with the following command. Possible values are: **debug**, **info**, **warn**, **error**
```xml
<arg name="log_level" value="info" />
``` 

The **topo_file_type** parameter tells the program which kind of topology is expected. DON'T CHANGE THIS LINE!
```xml
<arg name="topo_file_type" value="opil_sp" /> 
```
The **mars_vertex_footprint_radius** describes the size of the bounding box which is created by the topology launcher around each vertex. Important: This value must be smaller (**mars_vertex_footprint_radius** < **cell_size**) then the **cell_size** of SP! Value is in meter.
```xml
<arg name="mars_vertex_footprint_radius" value="0.95" />
```

#### Router
The Routing module which calculates the path for each robot can be started without any additional configuration. 
```xml
  <!-- ****** Router ***** -->
  <include file="$(find mars_routing_base)/launch/mars_routing_base.launch" />
```

#### Logical Agent(s)
Each RAN of the system is represented by a logical agent. The logical agents manages the high level tasks, like receiving and managing transport orders from the TS. For version 3.0.0-aplha, one AGV is preconfigured. 

```xml
  <!-- ****** Logical Agent (robot_0) ***** -->
  <include file="$(find mars_agent_logical_agv)/launch/mars_agent_logical_agv.launch">
    <arg name="physical_robot_namespace" value=""/>
    <arg name="robot_name" value="robot_opil_v2" />
    <arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
    <arg name="physical_agent_description" value="robot_0" />
    <!-- p0 -->
    <arg name="current_topology_entity_id" value="0a8b9081-d84c-5660-909c-134d55bf4966" />
    <arg name="node_name" value="ran_00000000000000000000000000000001" />
  </include>
```

The parameter **physical_robot_namespace** configures the namespace used by the ran. For e.g.: /opil/iot/ (In v3.0.0-alpha no namespace is configured)
```xml
<arg name="physical_robot_namespace" value=""/>
```

Name of the robot. Used to subscribe to the right robot topics. Layout of the topic structure: /physical_robot_namespace/robot_name/topics
```xml
<arg name="robot_name" value="robot_opil_v2" />
```

UUID of the physical agent. Id can be created randomly (UUID v4) or from a name (UUID v5). Important: The ID must be unique! For more information about UUID visit: https://en.wikipedia.org/wiki/Universally_unique_identifier
```xml
<arg name="physical_agent_id" value="00000000-0000-0000-0000-000000000001" />
```

Human readable name of the robot.
```xml
<arg name="physical_agent_description" value="robot_0" />
```

ID of the current node or edge on which the robot is located. Node end edge names are generated by SP. To translate a node oder edge name into a uuid, use UUID v5.
```xml
<arg name="current_topology_entity_id" value="0a8b9081-d84c-5660-909c-134d55bf4966" />
```

Name of the node. Topics and service are published as followed: /namespace/node_name/topic|service
```xml
<arg name="node_name" value="ran_00000000000000000000000000000001" />
```

### Configuration of TS
Following the configuration of the TS (ts_fiware_config.ini):

```ini
[flask]
host = 0.0.0.0

[taskplanner] 
host = <local network address>
PORT = 2906 

[contextbroker]
host= <orion address>
port=1026

[robots]
# these ids needs to be comma seperated, currently only robot 1 is used
# if you have multiple robots define the ids like that:
# ids = ran_00000000000000000000000000000001, ran_00000000000000000000000000000002, ... 
ids = ran_00000000000000000000000000000001
```