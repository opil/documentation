# <a name="CentralSPInstallation">Central SP Installation</a>

There are two ways of installing the SP module. You can use the docker containers or compile it from the source.


## Install from Docker Container

There are two docker containers - the Central SP and the Local SP.
The Central SP is located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp.central>

### The Central SP docker 



To install it you need to prepare a docker-compose.yml following this example:
### <a name="dockercompose">docker-compose.yml</a>

```
version: "3"
services:      
    mongo:
        image: mongo:3.4
        command: --nojournal

    ### Proxy for Context Broker ###
    ngsiproxy:
        image: fiware/ngsiproxy:latest
        ports:
            - 3000:3000

    ### Context Broker ###
    orion:
        image: fiware/orion:2.3.0
        depends_on:
            - mongo
            - ngsiproxy
        ports:
            - 1026:1026
        command:
            -dbhost mongo -corsOrigin __ALL -inReqPayloadMaxSize 2097152
#S&P
    sp:
        restart: always
        image: l4ms/opil.sw.sp.central:3.0.5-beta
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./annotations.ini:/annotations.ini:ro
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./topology.launch:/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - DISPLAY=$DISPLAY
```
This example uses the version 3 and it does not need links to enable services to communicate. It is assumed that for testing all services will be on the same machine (localhost). If orion is started on another machine, then environment variable of sp needs to have changed FIWAREHOST to IP address of that machine. 
When using the Central SP on big floorplans the parameter ***inReqPayloadMaxSize*** can be set to increase the maximal allowed size of any outgoing request messages. Setting it to 2097152 Bytes (2MB) should be enough for an average map with 30% occupied space with dimensions 65 m x 35 m and the resolution of the topology **cell_size** = 1 m. This setup requires 1327391 Bytes, while, by decreasing the **cell_size** to 0.5, requires 6687765 Bytes (approximately 4 times more). Similar increase of message size happens if your map is increased two times in width and height, for example 130 m x 70 m, with the **cell_size** = 1 m will require 6682759 Bytes (also 4 times more).


To update the docker-compose to the working version for the version 3 (1.22) type: (NOTE: you should remove prior versions of docker-compose)
```
sudo curl -L https://github.com/docker/compose/releases/download/1.22.0/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
```
and then
```
chmod +x /usr/local/bin/docker-compose
```
To check the version type:
```
sudo docker-compose --version
```
You should see:
```
docker-compose version 1.22.0, build f46880fe
```

First, set up the display for viewing the rviz visualization and Stage:
```
xhost local:root
```
This needs to be called only once.
Then, start it from the folder where you put your docker-compose.yml file:
```
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Central SP.](#fromdocker)






## Install from Scratch 

To install Firos v2:

```git clone -b v0.2.0 --recursive https://github.com/iml130/firos.git```


Install ROS packages:

```sudo apt-get install ros-kinetic-navigation```

```sudo apt-get install ros-kinetic-gmapping```

```sudo apt-get install ros-kinetic-teleop-twist-keyboard```

Install from SourceCode:

* put all ROS packages of Sensing and Perception to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace in src folder. Then compile it with catkin_make in one folder up.
```
cd ..
catkin_make
```
To check if everything is working properly follow the guide [Starting from Scratch.](#fromscratch)

# SP Server Start Guide

<!--Before starting the Central SP module follow the [Install guide](#CentralSPInstallation).-->


In the following, two start options will be described, depending if you installed the source from the docker containers or from scratch.


## <a name="fromdocker">Starting from Docker - Central SP</a>

Go to the folder where you put your [docker-compose.yml](#dockercompose) file and type:
```
xhost local:root
sudo docker-compose up
```

This docker container starts the creation of a topology graph from the given map file and with included annotations from the annotation file. As a default, no map is loaded and the following message will appear in terminal:
```
sp_1     | please insert a floorplan.png and floorplan.yaml file to begin!
sp_1     | in your docker-compose.yml put under volumes:
sp_1     |             - ./floorplan.yaml:/map.yaml:ro
sp_1     |             - ./floorplan.png:/map.png:ro
opilserver_sp_1 exited with code 0
```

### <a name="prepmap">Setting the map</a>

Two files need to be prepared and put under **volumes** in [docker-compose.yml](#dockercompose): 

*	`floorplan.yaml`
*	`floorplan.png` 

Right now in this docker container only PNG file is supported. Export your floorplan layout to PNG. 
Here is an example of `floorplan.png`:

![PNG floorplan](./img/teromap.png)

Then, you need to set up parameters for transforming the PNG file into a map with the origin and dimensions in meters. This needs to be prepared in the `floorplan.yaml`. Here is an example:
```
#floorplan.yaml
image: map.png
resolution: 0.0196
origin: [-12.7095, -7.5866, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
where

* the image name is `map.png` by which your `floorplan.png` is overwritten on the docker side
* **resolution** defines the size of the pixel of the png file in meters. To calculate the resolution, you need to know the width of the image in meters. Then, simply divide the width in meters with the width in the number of pixels. 
* **negate** can inverse the free-occupied values if set to 1
* **occupied_thresh** and **free_thresh** define the thresholds that tell which brightness level should be considered as occupied and free, respectively. 
* **origin** is the (x,y,z) vector in meters describing the lower-left corner. With (0,0,0) the origin will be in the lower-left corner. If the origin is somewhere inside the map, the origin coordinates will always have negative sign, since they represent the coordinates of the lower-left corner.

The files `floorplan.yaml` and `floorplan.png` need to be in the folder where you put your docker-compose.yml.
After restarting docker-compose.yml, i.e.,
```
sudo docker-compose up
```
 this is what should be the result:

![topology 2m](./img/terotopology.png)

You should see the entities in, e.g. firefox, at the address <http://localhost:1026/v2/entities>. There should be a topic `/map/graph`.
In this example, blue squares are the vertices of the graph placed regularly in the grid (distanced 2 meters), and blue line segments are the edges of the graph. Red squares are possible vertex positions which are occupied.
To zoom the rviz window use mouse scroll, to translate the view press shift key and mouse button at the same time.


### Setting the topology cell_size

It can be seen that here the topology nodes are too rare and we are missing some of them in narrow passages. In the following we will change the size of the grid cell so that we do not miss topology nodes in the passages between the racks (however, this will be solved differently in future version of OPIL). In this example map the passage is 2.5 m wide, which means the size of the cell size should be half of it to include the worst case of the alignment of passages with the grid.
To change the size of the grid cell for calculating the topology, prepare the `topology.launch` file:

### <a name="topologylaunch">topology.launch</a>
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/map.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 
<node name="map2gm" pkg="maptogridmap" type="map2gm" output="screen">
        <param name="cell_size" type="double" value="1.0" />
        <param name="annotation_file" textfile="$(find maptogridmap)/launch/annotations.ini" />
</node>
    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />
</launch>
```
where the **cell_size** is set to 1.0 m. 
To use the created topology.launch put it next to the docker-compose.yml file and restart docker-compose.yml.
This is the result for changing the parameter **cell_size** to 1.0 m:

![topology 1m](./img/topologytero1m.png)

### <a name="prepannt">Setting the annotations</a>

To use arbitrary annotations you should create `annotations.ini` file and put it in the same folder next to the docker-compose.yml:

```
#annotations.ini
[W1]
# coordinates
point_x = -5.4
point_y = -6.0
theta = 270
distance = 1.8

[W2]
# coordinates
point_x = -4.5
point_y = -6.0
theta = 270
distance = 1.8

[W3]
# coordinates
point_x = -3.6
point_y = -6.0
theta = 270
distance = 1.8

[MoldingPallet]
# coordinates
point_x = -6.0
point_y = 10.5
theta = 90
distance = 1.8
```
where

* **W1**, **W2**, **W3**, **MoldingPallet** are the annotation labels
* **point_x**, **point_y** are coordinates of the annotation
* **theta** and **distance** determine where the topology vertex should be so that Task Planner can use this coordinates as the goal distanced for a defined **distance** from the annotation and oriented towards the annotation so that AGV has heading **theta** with respect to the positive x-axis. 

Coordinates of the annotations can be chosen arbitrary. They can be set as the middle of the pallet or the corner of the pallet, for example. The most important is to calculate the right position where an AGV will be placed in front of the annotation by setting the right distance, and orientation from the annotation coordinates.

After restarting docker-compose.yml,
the result should be as in this figure:

![topology with annotations](./img/terotopologyannot.png)

Four annotations are in this example, which change the coordinates of vertices where the AGV needs to be located to perform some operation, e.g., loading/unloading.



## How to start the Local and Central SP docker containers together

Prepare a following docker-compose.yml to start both Central and Local SP. This example uses the same machine for Local and Central SP, but you can also start them on different machines.
This example uses the Local SP that contains the Stage simulator. Check how to prepare the Local SP [here](./opil_local_sp_install.md)

```
version: "3"
services:      
    #Context Broker
    orion:        
        image: fiware/orion
        ports:
            - 1026:1026
        command: 
            -dbhost mongo
    mongo:
        restart: always
        image: mongo:3.4
        command: --nojournal    
#S&P
    sp:
        restart: always
        image: l4ms/opil.sw.sp.central:3.0.5-beta
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./annotations.ini:/annotations.ini:ro
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./topology.launch:/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - DISPLAY=$DISPLAY
    splocal:
        restart: always
        image: l4ms/opil.iot.sp.local:3.0.5-beta
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./amcl.launch:/amcl_map.launch:ro
            - ./floorplan.world:/map.world:ro
            - ./local_robot_sim.launch:/local_robot_sim.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - DISPLAY=$DISPLAY
            - SIMULATION=true
```

The following figure presents the output after moving the green box in the Stage simulator:
![Local and central SP](./img/teromaplocalcentral.png)

One rviz window is from the Local SP, where you can see the AGV's pose (red arrow) and the local updates (red tiny squares). Another rviz window is from the Central SP, where you can see the updates of the topology and new obstacles presented with blue tiny squares showing only the current position of the new obstacle. Vertices at the position of a new obstacles are removed from the topology (blue squares become red, and connections are removed). All new obstacles are processed as they are received so only new ones are sent. That is the reason why in the Local SP you can see a trail of the obstacle, while in the Central SP there is no fine obstacle trail but only the topology nodes changes.




## <a name="fromscratch">Starting from Scratch</a>

In this quick guide, everything will be started on the same computer where you installed ROS and this source code. For advanced configuration of starting at different computers check [Interfaces](./../../develop/SP/opil_sp_interfaces.md).

* Start the map by launching the map_server package:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
```


You can also prepare your own simulation following [the guide for preparing the map](#prepmap) with slight difference pointed out here.

Create or edit the following files in the specified paths:

*	`maptogridmap/launch/floorplan.yaml`
*	`maptogridmap/launch/floorplan.png`
*	`maptogridmap/launch/startmapserver.launch`
*	`maptogridmap/launch/startmaptogridmap.launch`
*	`maptogridmap/launch/annotations.ini`

where

```
#floorplan.yaml
image: floorplan.png
resolution: 0.0196
origin: [-12.7095, -7.5866, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Note that in contrast to previously defined map files, here the **image** needs to point to the created floorplan.png. PGM files are also supported!

`startmapserver.launch` needs to have the right path to the created map files:
 
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/floorplan.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 

</launch>
```

Edit the `annotation.ini` file as explained in Section [Setting the annotations.](#prepannt)

* Start the topology calculation:
```
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
The result should be the started rviz with the map and topology with annotations as presented here:
![IML topology](./img/IMLtopology.png)
You can check the created topology in the topic graph by typing: 
```
rostopic echo /map/graph
```

You can change the resolution of the global gridmap and topology (the distance between the nodes) by checking the Section [Topology](./../../develop/SP/opil_api_sp.md#topology). 


To test if entities are sent to OCB, prepare the following docker-compose.yml on the machine where you want to have the OPIL server:
```
version: "3"
services:      
    #Context Broker
    orion:        
        image: fiware/orion
        ports:
            - 1026:1026
        command: 
            -dbhost mongo
            
    mongo:
        restart: always
        image: mongo:3.4
        command: --nojournal
```

To test sending topology to OCB put in firos/config all json files described in [Interfaces for Central SP](./../../develop/SP/opil_sp_interfaces.md#sp).
Set the right IP addresses for your machine (endpoint), OPIL server (contextbroker) in firos/config/config.json. This example uses the local configuration:
```
{
  "environment": "fof_lab",

  "fof_lab": {
    "server": {
        "port"      : 10103
    },
    "contextbroker": {
        "address"   : "127.0.0.1",
        "port"      : 1026,
        "subscription": {
          "throttling": 0,
          "subscription_length": 300,
          "subscription_refresh_delay": 0.5
        }
    },
    "endpoint": {
      "address": "127.0.0.1",
      "port": 39002
    },
    "log_level": "INFO"
  }
}
```
Run firos in a new terminal and check entities in a web browser at the address <http://OPIL_SERVER_IP:1026/v2/entities>.
```
terminal 5: rosrun firos core.py 
```

You can also start a single launch file which starts map_server, topology creation, and firos (this file is used in a docker container of Central SP):
```
roslaunch maptogridmap topology.launch
``` 
For more examples on sending and receiving these topics through OCB check the Section [Examples](./../../develop/SP/opil_api_sp.md#examplesOCB).

# Deinstallation
There is currently no deinstallation how-to. Simply remove all unused docker images and remove the source code.

# Upgrades
This is the first appearance of SP module so there is no upgrade procedure.

# Deprecated Features

Deprecated features are features that Central SP still supports but that are not maintained or evolved any longer, or will be used in the future. In particular:


## Firos v1 did not allow arrays of custom ROS messages

The topology is composed of nodes and edges. Since Firos is not supporting arrays of custom ROS messages it is divided into two ROS messages: nodes and edges.

Nodes.msg

	Header header	# standard ROS header
	nav_msgs/MapMetaData info	# number of cells in x and y directions of the gridmap, the size of the cell
	float64[] x	# x coordinate of the cell centre
	float64[] y	# y coordinate of the cell centre
	float64[] theta # orientation of the node used for annotations
	string[] name	# e.g. vertex_0 or annotation name
	string[] uuid	# unique id of a node
	
Edges.msg

	Header header	# standard ROS header 
	string[] uuid_src	# unique id of a source node of the edge
	string[] uuid_dest	# unique id of a destination node of the edge
	string[] name	# e.g. edge_0_1
	string[] uuid	# unique id of an edge

## Sending larger data on demand - a service mockup
Since there is no service call supported yet in Firos, topic _do_serve_ is used as a service mockup. On another machine that wants to obtain the data, on topic _do_serve_ needs to be sent value "true" or 1. Large data are a) map topic created with _map_server_ from PNG or PGM file and b) gridmap topic created from the map by resampling to cells of size given by the parameter _cell_size_.

### Testing sending map topic on request on machine_1:
Remark: only a small map can be tested here around 10m x 7m - Andamapa.yaml.
To test the sending of map topic the _mux_topicsmap.py_ program needs to be started which sends the map on the whitelisted topic only when master requests.
Start all these steps:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
terminal 3: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_1)
```
(the next two steps are optional, e.g., it will work without terminal 4 and 5) 
```
terminal 4: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 5: rosrun sensing_and_perception pubPoseWithCovariance
terminal 6: rosrun maptogridmap mux_topicsmap.py
terminal 7: rostopic pub /map/do_serve std_msgs/Bool '{data: 1}'
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "map" the topic "realmap".

### Testing sending gridmap topic on request:
To test the sending of gridmap topic the _mux_topics.py_ program needs to be started which sends the gridmap on the whitelisted topic only when master requests.
Start all previous steps (terminal 1, ..., terminal 7), but it works with starting only terminals 1,2,3.
```
terminal 8: rosrun maptogridmap mux_topics.py
terminal 9: rostopic pub /map/do_serve std_msgs/Bool '{data: 1}'
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "map" the topic "realtopology".


### Testing if topics are received on machine 2 through firos
```
terminal 1: roscore
terminal 2: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_2)
```
_maptogridmap_ package needs to be on the machine_2 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo these ros topics:
```
rostopic echo /map/realtopology (if test sending gridmap is on)
rostopic echo /map/realmap (if test sending map is on)
```




