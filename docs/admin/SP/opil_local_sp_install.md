

# Local SP Installation

There are two ways of installing the SP module. You can use the docker containers or compile it from the source.

## Install from Docker Container

There are two docker containers - the Central SP and the Local SP.
The Local SP is located at 
<https://hub.docker.com/r/l4ms/opil.iot.sp.local>

### The Local SP as a standalone module


The Local SP docker container can be started in two ways: without RAN and with RAN. If started without RAN then the simulator Stage is used for testing and visualizing what the Local SP does. Here, installing the Local SP as a standalone module is described.

To install it you need to prepare a docker-compose.yml following this example:
### <a name="dockercomposelocal">docker-compose.yml</a>
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
    splocal:
        restart: always
        image: l4ms/opil.iot.sp.local:latest
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./amcl.launch:/amcl_map.launch:ro
            - ./floorplan.world:/map.world:ro
            - ./local_robot_sim.launch:/local_robot_sim.launch:ro
            - ./local_robot.launch:/local_robot.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
            - SIMULATION=true
```
It is important to put the environment variable `SIMULATION=true`.
First, set up the display for viewing the rviz visualization and Stage:
```
xhost local:root
```
This needs to be called only once.
Then, start it from the folder where you put your docker-compose.yml file:
```
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Local SP.].

## The Local SP docker working with RAN

TODO: a new docker container of RAN needs to be created since the entities have changed. This is old and as a place holder.

The Local SP docker container is the same but the environment variable needs to be `SIMULATION=false`. By this, the Local SP does not start the Stage simulator and it is connected directly to RAN through a single ROS master. For that purpose RAN docker container is called with the environment variable `SIMULATION=3`, which does not call **map_server** and **amcl** localization.

To install it you need to prepare a docker-compose.yml that also includes RAN container:
### <a name="dockercomposelocalran">docker-compose.yml</a>
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
        image: l4ms/opil.iot.ran:sp1
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./robot_launcher.launch:/root/catkin_ws/src/opil_v2/robot_launcher_with_SandP.launch:ro
#            - ./testmap.png:/root/catkin_ws/src/opil_v2/map/map.png:ro
#            - ./testmap.world:/root/catkin_ws/src/opil_v2/map/map.world:ro
        environment:
            - FIWAREHOST=orion
            - HOST=ran
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
            - SIMULATION=3

#S&P
    splocal:
        restart: always
        image: l4ms/opil.iot.sp.local:latest
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./amcl.launch:/amcl_map.launch:ro
            - ./floorplan.world:/map.world:ro
            - ./local_robot_sim.launch:/local_robot_sim.launch:ro
            - ./local_robot.launch:/local_robot.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
            - SIMULATION=false
```
Then, start it from the folder where you put your docker-compose.yml file:
```
sudo docker-compose up
```
To check if everything is working properly follow the guide [How to start the Local SP and RAN docker containers together.](#LocalSPStartGuide)




## Install from Scratch 

To install Firos v2:

```git clone --recursive https://github.com/iml130/firos.git```


Install ROS packages:

```sudo apt-get install ros-kinetic-navigation```

```sudo apt-get install ros-kinetic-gmapping```

```sudo apt-get install ros-kinetic-teleop-twist-keyboard```

Install from SourceCode:

* put everything to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace in src folder. Then compile it with catkin_make in one folder up.
```
cd ..
catkin_make
```
To check if everything is working properly follow the guide [Starting from Scratch.](#SPStartGuide)

# <a name="SPStartGuide">Local SP Start Guide</a>


Before starting the Local SP module follow the [Install guide](#SPStartGuide).


In the following, two start options will be described, depending if you installed the source from the docker containers or from scratch.


## <a name="fromdockerlocal">Starting from Docker - Local SP</a>

The Local SP container can be started in two ways: without RAN and with RAN. If started without RAN then the simulator Stage is used for testing and visualizing what the Local SP does. This option is described first. Prepare [docker-compose.yml](./opil_local_sp_install.md#dockercomposelocal).
It is important to put the environment variable `SIMULATION=true`.

As a default, no map is loaded and the following message will appear in terminal:
```
splocal_1  | please insert a floorplan.png and floorplan.yaml file to begin!
splocal_1  | in your docker-compose.yml put under volumes:
splocal_1  |             - ./floorplan.yaml:/map.yaml:ro
splocal_1  |             - ./floorplan.png:/map.png:ro
opilserver_splocal_1 exited with code 0
```

### <a name="prepmap">Setting the map</a>

Three files need to be prepared and put under **volumes** in [docker-compose.yml](./opil_server_sp_install.md#dockercompose): 

*	`floorplan.yaml`
*	`floorplan.png` 
*	`floorplan.world`

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
* **origin** is the (x,y,z) vector in meters describing the lower-left corner. With (0,0,0) the origin will be in the lower-left corner.

`floorplan.world` is the world file for the Stage simulator, where example is given as follows:
```
define kinect ranger
(
  sensor
  (
    range [0.0 10]
    fov 360.0
    samples 720
  )

  # generic model properties
  color "black"
  size [ 0.060 0.150 0.030 ]
)

define robot position
(
  origin [ 0.000 0.000 0.000 0.000 ]
  localizaton "odom"
  odom_error [0.03 0.03 0.01 0.01 0.02 0.02] 
  size [ 0.600 0.400 0.400 ]  
  drive "omni"
    
  kinect(pose [ -0.100 0.000 -0.110 0.000 ])
)
#define an obstacle
define worker model
(
  size [0.500 0.500 1.850]
  gui_nose 0  
)

define floorplan model
(
  color "gray30"  
  boundary 0
  gui_nose 0
  gui_grid 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)

resolution 0.05

# simulation timestep in milliseconds
interval_sim 100  

#stage view window
window
(
  size [ 935 412 ]
  center [ 18.080 20.285 ]
  rotate [ 0.000 0.000 ]
  # pixel per meter
  scale 13.110
)

# stage map
floorplan
(
  name "floorplan"
  bitmap "map.png"
  size [ 29.40 21.168 1.000 ]
  pose [ 1.9905 2.9974 0.000 0.000 ] 
)

# throw in a robot
robot
(
  pose [ 1.0 0 0.000 0.000 ] 
  name "robot_0"
  color "blue"  
)

worker
(
	pose [ -2.538 1.380 0.000 180.000 ]
	color "green"
)
```
The most important parameters are **size** and **pose** in the **floorplan** section which need to be calculated in meters from `floorplan.png` and **resolution** in `floorplan.yaml`. For example, our `floorplan.png` has 1500 x 1080 pixels, and in `floorplan.yaml` we see that resolution is 0.0196. The first column in **size** is the width of the map along **x** axis in meters obtained by multiplication of resolution and width in pixels, i.e., 1500 x 0.0196 = 29.4 m, while the second column in **size** is the height along **y** axis obtained by multiplication of resolution and height in pixels, i.e., 1080 x 0.0196 = 21.168 m. The third column is the height along **z** axis of the floorplan, i.e. 1 meter. **pose** of the **floorplan** defines the origin of the loaded map in Stage in the form of **x**, **y**, **z**, **theta** (in degrees) values. If values for **x** and **y** coordinates are half of the **floorplan**'s **size**, then the origin will be in the lower left corner of the map. Therefore, to have aligned coordinate systems of Stage and map we need to sum floorplan's pose and yaml's origin. In our case it is 29.4/2 -12.7095 = 1.9905 for x coordinate, and 21.168/2 -7.5866 = 2.9974. Choose the **pose** of the **robot**. It should be aligned to the initial pose in [amcl.launch](#amcllaunch). Choose the pose of the **worker**. It is here treated as moving obstacle. The rest of the parameters can be the same as in this example. For more explanation about the **world** files visit <http://wiki.ros.org/stage>. 

### Setting the initial pose for the localization

Parameters of **amcl** localization can be set in `amcl.launch`, and here is a default file:
### <a name="amcllaunch">amcl_testmap.launch</a>
```
<launch>
<node pkg="amcl" type="amcl" name="amcl" respawn="true">
<remap from="scan" to="base_scan" />
  <!-- Publish scans from best pose at a max of 10 Hz -->
  <param name="odom_model_type" value="omni"/>
  <param name="odom_alpha5" value="0.1"/>
  <param name="transform_tolerance" value="0.2" />
  <param name="gui_publish_rate" value="10.0"/>
  <param name="laser_max_beams" value="24"/>
  <param name="min_particles" value="500"/>
  <param name="max_particles" value="5000"/>
  <param name="kld_err" value="0.05"/>
  <param name="kld_z" value="0.99"/>
  <param name="odom_alpha1" value="0.2"/>
  <param name="odom_alpha2" value="0.2"/>
  <!-- translation std dev, m -->
  <param name="odom_alpha3" value="0.8"/>
  <param name="odom_alpha4" value="0.2"/>
  <param name="laser_z_hit" value="0.5"/>
  <param name="laser_z_short" value="0.05"/>
  <param name="laser_z_max" value="0.05"/>
  <param name="laser_z_rand" value="0.5"/>
  <param name="laser_sigma_hit" value="0.2"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_model_type" value="likelihood_field"/>
  <!-- <param name="laser_model_type" value="beam"/> -->
  <param name="laser_likelihood_max_dist" value="2.0"/>
  <param name="update_min_d" value="0.2"/>
  <param name="update_min_a" value="0.05"/>
  <param name="odom_frame_id" value="odom"/>
  <param name="resample_interval" value="1"/>
  <param name="transform_tolerance" value="0.1"/>
  <param name="recovery_alpha_slow" value="0.0"/>
  <param name="recovery_alpha_fast" value="0.0"/>
  <param name="initial_pose_x" value="1.0"/> 
  <param name="initial_pose_y" value="0"/>  
  <param name="initial_pose_a" value="0"/>
</node>
</launch>
```
The most important parameters are the range topic, which for simulator is **base_scan**, and initial values for the pose of the robot: **initial_pose_x**, **initial_pose_y**, **initial_pose_a** (angle in radians). For all other parameters visit <http://wiki.ros.org/amcl>.



All files need to be in the folder where you put your docker-compose.yml.
After restarting docker-compose.yml, i.e.,
```
sudo docker-compose up
```
 this is what should be the result:
![Local SP](./img/localsptero.png)

This starts the **amcl** localization inside the map shown on topic `/robot_0/pose_channel`, and calculation of map updates shown on topic `/robot_0/newObstacles`. The robot in the Stage simulator can be moved by mouse dragging, and changed pose can be seen in topic `/robot_0/pose_channel` which shows the current pose and the covariance of the pose estimation. There is also an obstacle in the Stage simulator that can be used for test of showing the map updates, seen in topic `/robot_0/newObstacles`.
In the rviz window you can see the AGV's pose (red arrow) and local updates (red tiny squares). All new obstacles are processed as they are detected, and there is no clearance of obstacles since they are considered to be static. That is the reason why you can see a trail of the obstacle. In future developments static and dynamic obstacles will be treated differently.


The fine grid for calculation of map updates can be set by preparing the `local_robot_sim.launch` file, which has the following structure:
### <a name="localrobotsimlaunch">local_robot_sim.launch</a>
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find lam_simulator)/yaml/map.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node pkg="stage_ros" type="stageros" name="stageros" args="$(find lam_simulator)/yaml/map.world" respawn="false" >
    <param name="base_watchdog_timeout" value="0.2"/>
</node>
<include file="$(find lam_simulator)/launch/amcl_map.launch" />
     <!--- Run pubPoseWithCovariance node from sensing_and_perception package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
     <node name="publishPoseWithCovariance" pkg="sensing_and_perception" type="pubPoseWithCovariance" output="screen" args="0"/>	

     <!--- Run mapup node from mapupdates package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
    <node name="mapup" pkg="mapupdates" type="mapup" output="screen" args="0" >
        <param name="cell_size" type="double" value="0.1" />
        <param name="scan_topic" value="/base_scan" />
        <param name="laser_inverted" type="bool" value="0" />
    </node>

    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find lam_simulator)/rviz_cfg/singlerobot.rviz" /> 

</launch>
```
In this launch file you can set various parameters: **cell_size** defines the fine gridmap for calculating new obstacles (presented as tiny red squares); robot ID as an argument of packages _sending_and_perception_ and _mapupdates_ for having more robots (0,1,2,etc.), and **scan_topic** for defining the topic for the range data.
There is also a parameter **laser_inverted** to indicate if the laser is mounted upside down on the robot (use value 1), or normally (use value 0). The simplest check if laser is setup correctly is to see in rviz if laser readings are aligned with the png map.  
Right now it is implemented that Central SP can handle three robots with ID's 0, 1 and 2. If you change the ID of the robot to 1 the topics `/robot_1/newObstacles` and `/robot_1/pose_channel` will be created and visible in OCB.


## <a name="fromdockerlocalran">How to start the Local SP and RAN docker containers together</a>

The Local SP container needs to be started with the environment variable `SIMULATION=false`, as is the case in [docker-compose.yml](./opil_local_sp_install.md#dockercomposelocalran).

Again, you need to set up the map as explained [previously](#prepmap).

If you want to change the parameters for map updates prepare `local_robot.launch` as follows:
### <a name="localrobotlaunch">local_robot.launch</a>
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find lam_simulator)/yaml/map.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<include file="$(find lam_simulator)/launch/amcl_map.launch" />
     <!--- Run pubPoseWithCovariance node from sensing_and_perception package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
     <node name="publishPoseWithCovariance" pkg="sensing_and_perception" type="pubPoseWithCovariance" output="screen" args="0"/>	

     <!--- Run mapup node from mapupdates package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
    <node name="mapup" pkg="mapupdates" type="mapup" output="screen" args="0" >
        <param name="cell_size" type="double" value="0.1" />
        <param name="scan_topic" value="/base_scan" />
        <param name="laser_inverted" type="bool" value="0" />
    </node>

    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find lam_simulator)/rviz_cfg/singlerobot.rviz" /> 

</launch>
```

Then, prepare `robot_launcher.launch` as follows:
### <a name="robotlauncherlaunch">robot_launcher.launch</a>
```
<launch>
    <!-- Run FIROS -->  
    <node name="firos" pkg="firos" type="core.py" />
    <!-- Run the Stage simulation -->
    <node name="stageros" pkg="stage_ros" type="stageros" args="$(find opil_v2)/map/map.world" respawn="false"> 
        <param name="base_watchdog_timeout" value="0.2" />
    </node>
 
        <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
            <remap from="map" to="/map" />                      
            <param name="controller_frequency" value="10.0" />
            <rosparam file="$(find opil_v2)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
            <rosparam file="$(find opil_v2)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
            <rosparam file="$(find opil_v2)/config/global_costmap_params.yaml" command="load" />
            <rosparam file="$(find opil_v2)/config/local_costmap_params.yaml" command="load" />
            <rosparam file="$(find opil_v2)/config/base_local_planner_params.yaml" command="load" />
        </node>  
         <!--- Run the Robot Agent node -->
         <node name="robot_agent" pkg="opil_v2" type="opil_v2_robot_agent_with_SandP" output="screen" args="0"/>
    
</launch>
```
To use the changed robot_launcher.launch uncomment the line in [docker-compose.yml](./opil_local_sp_install.md#dockercomposelocalran) under **volumes**:
```
            - ./robot_launcher.launch:/root/catkin_ws/src/opil_v2/robot_launcher_with_SandP.launch:ro
```

The following example presents testing the ICENT map. The robot can be controlled through rviz by pressing 2D Nav Goal button and clicking the point on the map. The following figure describes the possible outcome, where arrow trails correspond to the past positions of the robot:
![Local SP and RAN in ICENT lab](./img/movingwithran.png)


## <a name="fromscratch">Starting from Scratch</a>

In this quick guide, everything will be started on the same computer where you installed ROS and this source code. For advanced configuration of starting at different computers check [Interfaces](./../../develop/SP/opil_sp_interfaces.md).

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
You can start some other simulation example following [this guide.](./../../develop/SP/opil_api_local_sp.md#poswithcov) 
You can also prepare your own simulation following [the guide for preparing the map.](./../../develop/SP/opil_api_local_sp.md#preparingmap)

* Start the calculation of local map updates that the AGV sees as new obstacles which are not mapped in the initial map.
```
terminal 3: roslaunch mapupdates startmapupdates.launch
```
You can check the topic of local map updates by typing: 
```
rostopic echo /robot_0/newObstacles
```
You can change the resolution of the local map updates by following the guide in Section [Map updates](./../../develop/SP/opil_api_local_sp.md#mapupdates).



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

To test sending local map updates and pose with covariance to OCB put in firos/config all json files from test/config_files/machine_2. Set the right IP addresses for your machine (server), OPIL server (contextbroker), and interface (check with ifconfig) in firos/config/config.json. Run firos in a new terminal and check entities in a web browser at the address <http://OPIL_SERVER_IP:1026/v2/entities>.
```
terminal 5: rosrun firos core.py 
```

All previous steps can be replaced by calling a single launch file for the Local SP:
```
roslaunch sensing_and_perception local_robot_sim.launch 
```
This launch file starts the localization, local map updates and module for publishing Pose with Covariance, and firos. The launch file that does not start the simulator Stage because it will be started at RAN is **local_robot.launch**.

For more details how to set up firos config files check the Section [Interfaces](./../../develop/SP/opil_sp_interfaces.md). For more examples on sending and receiving these topics through OCB check the Section [Examples](./../../develop/SP/opil_api_sp.md#examplesOCB).



# Deinstallation

There is currently no deinstallation how-to. Simply remove all unused docker images and remove the source code.


# Upgrades

This is the first appearance of SP module so there is no upgrade procedure.


# Local SP Deprecated 

Deprecated features are features that SP still supports but that are not maintained or evolved any longer, or will be used in the future. In particular:


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

##Interconnection between machines

### Towards Orion CB
To send the topics to Orion Context Broker json files needs to be set properly inside the firos/config folder and firos needs to be running.

### Towards Physical IO

This direction is not yet used, meaning that something is being sent to IO from OCB. There are two reasons: a) the map is too big; b) there is no service call so AMCL can not work.

But, in general, to send the topics through firos to Physical IO json files need to be set properly inside the firos/config folder and firos needs to be running.

### firos config json files explained between machine 1 and machine 2

On machine 1 all topics that are being sent through context broker need to be "subscriber", and that are being received from the context broker need to be "publisher". Topics are listed under ids and here we have "map" id and "robot_0" id.

The numbering in names robot_0, robot_1, etc. corresponds to the number of used AGVs and you should correct config files with respect to number of AGVs.

#### robots.json
```
{
   "map":{
       "topics": {
                "realtopology": {
                    "msg": "maptogridmap.msg.Gridmap",
                    "type": "subscriber"
                },
            	"realmap": {
                	"msg": "nav_msgs.msg.OccupancyGrid",
                	"type": "subscriber"

            	},
            	"nodes": {
                	"msg": "maptogridmap.msg.Nodes",
                	"type": "subscriber"

            	},
            	"edges": {
                	"msg": "maptogridmap.msg.Edges",
                	"type": "subscriber"

            	},
                "do_serve": {
                    "msg": "std_msgs.msg.Bool",
                    "type": "publisher"
                }
       }
   },
   "robot_0":{
        "topics": {
	    "pose_channel": {
                "msg": "geometry_msgs.msg.PoseWithCovarianceStamped",
                "type": "subscriber"
 	       }
	    }
	}

}
```
#### whitelist.json
```
{
    "map": {
        "subscriber": ["realtopology","nodes","realmap","edges"],
        "publisher": ["do_serve"]
    },
    "robot_0": {
        "subscriber": ["pose_channel"]
    }
}
```
On machine 2 there is exactly the opposite subscriber/publisher from machine 1, so simply just replace subscriber and publisher.



