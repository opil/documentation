# Start Guide

Before starting the Local SP module follow the [Install guide](./opil_local_sp_install.md).


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

To test sending local map updates and pose with covariance to OCB put in firos/config all json files from test/config_files/machine_2. Set the right IP addresses for your machine (server), OPIL server (contextbroker), and interface (check with ifconfig) in firos/config/config.json. Run firos in a new terminal and check entities in a web browser at the address <http://OPIL_SERVER_IP:1026/v2/entities.
```
terminal 5: rosrun firos core.py 
```

All previous steps can be replaced by calling a single launch file for the Local SP:
```
roslaunch sensing_and_perception local_robot_sim.launch 
```
This launch file starts the localization, local map updates and module for publishing Pose with Covariance, and firos. The launch file that does not start the simulator Stage because it will be started at RAN is **local_robot.launch**.

For more details how to set up firos config files check the Section [Interfaces](./../../develop/SP/opil_sp_interfaces.md). For more examples on sending and receiving these topics through OCB check the Section [Examples](./../../develop/SP/opil_api_sp.md#examplesOCB).
