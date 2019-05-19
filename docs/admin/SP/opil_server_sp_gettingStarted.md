# Start Guide

Before starting the SP module follow the [Install guide](./opil_server_sp_install.md).

<!--
 and [API Walkthrough](./user/api.md) procedures.
-->

In the following, two start options will be described, depending if you installed the source from the docker containers or from scratch.


## <a name="fromdocker">Starting from Docker - Central SP</a>

Go to the folder where you put your [docker-compose.yml](./opil_server_sp_install.md#dockercompose) file and type:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
You should see the entities in, e.g. firefox, at the address <http://localhost:1026/v2/entities>.

This docker container starts the creation of topology (nodes, edges) from the given map file, and with included annotations from the annotation file. As a default, IML lab will be started with 4 annotations, and with the default 2 m cell size, as is the case in this example:

![IML topology](./img/IMLtopology.png)

To use arbitrary annotations file and map file you should create these files and put it in the same folder next to the docker-compose.yml and uncomment the lines containing these files. These example files can be found in the folder test/docker_compose_files/Central_SP_docker.

Here is a quick guide how to create these files:

* To use arbitrary annotations create the **annotations.ini** file:
```
#annotations.ini
[P1]
# coordinates
point_x = 18.4
point_y = 6.5
theta = 180
distance = 1.8
```
where **point_x**, **point_y** are coordinates of the annotation, and **theta** and **distance** determine where the topology node should be so that Task Planner can use this node as the goal distanced (1.8 m in this example) from the annotation and oriented towards the annotation so that AGV has heading of **theta** (180 degrees in this example) with respect to the positive x-axis.

* Uncomment the line in the [docker-compose.yml](./opil_server_sp_install.md#dockercompose):
```
            - ./annotations.ini:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/annotations.ini:ro
```
After restarting docker-compose.yml, i.e., type:
```
sudo docker-compose up
```
the result should be as in this figure:

![IML topology](./img/IMLtopologyanntex.png)

* To use arbitrary map file you should prepare **map.png** (export as png in any graphical software) and **map.yaml**. Here is an example of the CHEMI factory floorplan saved as png file:

![IML topology](../img/map.png)

It should be a greyscaled image, where dark grey will be considered as obstacle, while light grey as free area.

* To set the dimensions of the map file you need to prepare parameter file **map.yaml**, where you need to set the **resolution** of the map, and thresholds for defining what will be considered as an obstacle, and what as a free area. Here is an example for the CHEMI factory:
```
image: map.png
resolution: 0.12
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.165
free_thresh: 0.001
```
Here, the line ```image: map.png``` should not be ever changed since, on the docker side, the name **map.png** will always be used, while the file names can be arbitrary since they are copied to the **map.png** and **map.yaml**. The parameter ```resolution: 0.12``` means the size of the pixel of the png file is 0.12 m wide. To calculate the **resolution**, you need to know the width of the image in meters. Then, simply divide the width of the image with the number of pixels. Adjust the parameters **occupied_thresh** and **free_thresh** to different values, depending on which shade of grey should be considered as occupied. In this example, thresholds are quite low, which means almost only white is considered as a free area.

* Uncomment the lines in the [docker-compose.yml](./opil_server_sp_install.md#dockercompose):
```
            - ./map.yaml:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.yaml:ro
            - ./map.png:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.png:ro
```
After restarting docker-compose.yml this is what should be the result:

![IML topology](./img/chemi2mgridcell.png)

It can be seen that here the topology nodes are too rare and we are missing some of them in narrow passages. In the following we will change the size of the grid cell so that we do not miss topology nodes in the passages between the racks. In this example map the passage is 2.5 m wide, which means the size of the cell size should be half of it to include the worst case of the alignment of passages with the grid.
 
* To change the size of the grid cell for calculating the topology prepare **topology.launch** as follows:
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/map.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 
<node name="map2gm" pkg="maptogridmap" type="map2gm" output="screen">
        <param name="cell_size" type="double" value="1.25" />
        <param name="annotation_file" textfile="$(find maptogridmap)/launch/annotations.ini" />
</node>
    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />
</launch>
```
where we put the parameter **cell_size** to 1.25 m.

* Uncomment the line in the [docker-compose.yml](./opil_server_sp_install.md#dockercompose):
```
            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
```
After restarting docker-compose.yml this is what should be the result:

![IML topology](./img/chemi1p25mgridcell.png)

## <a name="fromdockerlocal">Starting from Docker - Local SP</a>

The current version of the Local SP contains the Stage simulator, which will be removed in the future version since it will be connected to the RAN through ROS master.

Prepare a following docker-compose to start both Central and Local SP. You can also start them on the different machines.

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
        image: l4ms/opil.sw.sp:c2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
    splocal:
        restart: always
        image: l4ms/opil.sw.sp:l2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```

This will start everything with the IML lab. The following figure presents the output after moving the green box in the Stage simulator:
![Local and central SP](./img/localcentralupdateIML.png)

One rviz window is from the Local SP, where you can see the AGV's pose (red arrow) and local updates (red tiny squares). Another rviz window is from the Central SP, where you can see the updates of the topology and new obstacles presented with blue tiny squares presenting only the current position of the new obstacle. All new obstacles are processed as they are received so only new ones are sent. That is the reason why in the Local SP you can see a trail of the obstacle, while in the Central SP there is no trail but the topology is permanently changed.

You can test more maps, and here is the examle how to change the map to ICENT lab.
Prepare the file **local_robot_sim.launch** and put it next to your **docker-compose.yml**:

```
<launch>

<!--ICENT lab-->
<node name="map_server" pkg="map_server" type="map_server" args="$(find lam_simulator)/yaml/Andamapa.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node pkg="stage_ros" type="stageros" name="stageros" args="$(find lam_simulator)/world/andaomnidriveamcltest.world" respawn="false" >
    <param name="base_watchdog_timeout" value="0.2"/>
</node>

<!--IML lab
<node name="map_server" pkg="map_server" type="map_server" args="$(find lam_simulator)/yaml/IMLlab.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node pkg="stage_ros" type="stageros" name="stageros" args="$(find lam_simulator)/world/floormap_IML.world" respawn="false" >
    <param name="base_watchdog_timeout" value="0.2"/>
</node>
-->

<include file="$(find lam_simulator)/launch/amcl_omnisimdemo.launch" />

     <!--- Run pubPoseWithCovariance node from sensing_and_perception package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
     <node name="publishPoseWithCovariance" pkg="sensing_and_perception" type="pubPoseWithCovariance" output="screen" args="0"/>	

     <!--- Run mapup node from mapupdates package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
    <node name="mapup" pkg="mapupdates" type="mapup" output="screen" args="0" >
        <param name="cell_size" type="double" value="0.1" />
        <param name="scan_topic" value="/base_scan" />
    </node>

    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find lam_simulator)/rviz_cfg/singlerobot.rviz" /> 

</launch>
```
In this launch files you can set various parameters: **cell_size** defines the fine gridmap for calculating new obstacles (tiny red squares); robot ID as an argument of _pubPoseWithCovariance_ and _mapupdates_ for having more robots, **scan_topic**, etc.
For testing Local SP and Central SP on the same ICENT map, you also need to prepare a new **topology.launch**:
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/Andamapa.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 
<node name="map2gm" pkg="maptogridmap" type="map2gm" output="screen">
    <param name="cell_size" type="double" value="1.25" />
    <param name="annotation_file" textfile="$(find maptogridmap)/launch/annotations.ini" />
</node>
<!-- Run FIROS -->
<node name="firos" pkg="firos" type="core.py" />
</launch>
```
Now the **docker-compose.yml** should include these two files as volumes:
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
        image: l4ms/opil.sw.sp:c2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
    splocal:
        restart: always
        image: l4ms/opil.sw.sp:l2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./local_robot_sim.launch:/root/catkin_ws/src/mod.sw.sp/src/localization_and_mapping/sensing_and_perception/local_robot_sim.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
After moving the green box in the simulator the result can be seen like in this figure:
![Local and central SP in ICENT lab](./img/localcentralupdate.png)

## <a name="fromdockerlocal">Starting from Docker - Local SP and RAN</a>

This section explains connecting Local SP, where Local SP does not contain the Stage simulator and it is connected directly to RAN through a single ROS master (has tag l3.0). For that purpose RAN docker container is changed so that it does not contain map_server and amcl localization and right now temporal version is used from opilsp/ran:3.0.

Prepare a following docker-compose to start both Central and Local SP:

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
    ran:
        restart: always
        image: opilsp/ran:3.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        environment:
            - FIWAREHOST=orion
            - HOST=ran
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY

#S&P
    splocal:
        restart: always
        image: l4ms/opil.sw.sp:l3.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
ICENT map will be started and you can command the robot through rviz by pressing 2D Nav Goal button and clicking the point on the map. The following figure describes the possible outcome, where arrow trails correspond to the past positions of the robot:
![Local SP and RAN in ICENT lab](./img/movingwithran.png)

You can also add Central SP in the docker-compose and obtain results similar to the previous section.

## <a name="fromscratch">Starting from Scratch</a>

In this quick guide, everything will be started on the same computer where you installed ROS and this source code. For advanced configuration of starting at different computers check [Interfaces](../../develop/SP/opil_sp_interfaces.md).

* First, start the Stage simulator with the loaded example map and one AGV inside the map (red) and one box as the unknown obstacle (green):

```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
```
The result should be the started rviz and the Stage simulator as presented here:
![SP module architecture](./img/localizationAndamap.png)

The green dots are simulated laser readings, and the red arrow is the localized AGV.

* Then, start the module for creating the topic for Pose with Covariance of the AGV:
```
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```
You can type 'rostopic echo' to see the message of pose with covariance:
```
rostopic echo /robot_0/pose_channel
```
You can start some other simulation example following [this guide.](../../develop/SP/opil_api_sp.md#mapupdates#poswithcov)
You can also prepare your own simulation following [the guide for preparing the map.](../../develop/SP/opil_api_sp.mdpreparingmap)

* Start the calculation of local map updates that the AGV sees as new obstacles which are not mapped in the initial map.
```
terminal 3: roslaunch mapupdates startmapupdates.launch
```
You can check the topic of local map updates by typing: 
```
rostopic echo /robot_0/newObstacles
```
You can change the resolution of the local map updates by following the guide in Section [Map updates]../../develop/SP/opil_api_sp.md#mapupdates).

All previous steps can be replaced by calling a single launch file for the Local SP:
```
roslaunch sensing_and_perception local_robot_sim.launch 
```
This launch file starts the localization, local map updates and module for publishing Pose with Covariance. The launch file that does not start the simulator Stage because it will be started at RAN is **local_robot.launch**.

* To merge the local map updates into a global gridmap and topology, launch the maptogridmap:
```
terminal 4: roslaunch maptogridmap startmaptogridmap.launch
```
You can change the resolution of the global gridmap and topology (the distance between the nodes) by checking the Section [Topology](../../develop/SP/opil_api_sp.md#topology). There you can also find about annotations and how to change theirs parameters.

Now, you can reproduce the figures from [Home](./index.md#mapupdates1) by moving the green box in front of the AGV (use the left click of the mouse and hold it while dragging the box around).
