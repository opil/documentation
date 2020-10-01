In the following the list of ROS packages is given with the short explanation.
Afterwards, each package is explained with the followed examples.

# Overview of ROS packages for Local SP

All ROS packages are in src folder. Here are explained only the ones belonging to the Local SP.


## mapupdates package

A ROS package for creating local map updates from the range data (laser or kinect readings). This package is located on an AGV. An AGV needs to start this package with the robot ID number set to 0, 1, etc., so that published topic has corresponding name as /robot_0/newObstacles, /robot_1/newObstacles, etc.

## maplistener package

A ROS package for testing subscribers to all created topics from mapupdates and visualizing them in rviz.


## localization_and_mapping metapackage

A ROS metapackage containing packages for localization and mapping listed in the following: 


### localization_and_mapping packages

#### lam_simulator

ROS package which demonstrates localization and mapping in the Stage simulator. Relies on AMCL or gmapping algorithms started using the Stage simulator. Prepared demonstrations can also be used on the real robot.


#### sensing_and_perception

ROS package for publishing AGV's pose with covariance to be sent to Orion Context Broker through firos.

#### andymark_driver

ROS drivers for omnidirectional Andymark platform. Teleoperation and control relies on the nodes provided by the _husky_ package.


#### husky

ROS package for interfacing with Clearpath Husky robot. It also includes nodes for teleoperation and control.


#### odometry_correction

ROS package which relies on _robot_pose_ekf_ to fuse robot odometry with IMU data to improve the odometry estimation.






# <a name="poswithcov">Pose with covariance</a>

A standard ROS message is used for sending the pose of the AGV:
	
	geometry_msgs.msg.PoseWithCovarianceStamped
	
To be able to send this message, two modules needs to be started: the AMCL package and the sensing_and_perception package. The AMCL package provides localization of the AGV inside a given map using the odometry and laser sensors. The sensing_and_perception package combines the covariance calculated by the AMCL package and the the global pose in the map frame as a result of a combination of odometry and AMCL localization with the laser.
Since this topic will be send through firos to OCB, the unique robot id needs to be set in the lauch file through _args_, e.g. 0, 1, 2, ... In this example launch file send_posewithcovariance.launch you can see that the id of the robot is set to 0. This creates a topic `/robot_0/pose_channel`. If you change the ID of the robot to 1 the topic `/robot_1/pose_channel` will be created:
```
<launch>

     <!--- Run pubPoseWithCovariance node from sensing_and_perception package-->
     <!-- Put args="1" if you are testing the robot with the id number 1 -->
     <node name="publishPoseWithCovariance" pkg="sensing_and_perception" type="pubPoseWithCovariance" output="screen" args="0"/>	

</launch>
```

Example with the MURAPLAST factory floorplan:
```
terminal 1: roslaunch lam_simulator amcl_test_muraplast.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```

Example with the ICENT lab floorplan in Zagreb review meeting demo:
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```

Example with the IML lab floorplan:
```
terminal 1: roslaunch lam_simulator IMLamcltest.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```
As the result you can echo the topic /robot_0/pose_channel:
```
rostopic echo /robot_0/pose_channel
```
If you want to send this topic through firos, use and adapt the config files in firos_config inside the localization_and_mapping metapackage (or from test/config_files/machine_1) and start firos by typing:
```
terminal 3: rosrun firos core.py 
```

# SLAM

When the map (or we also say the initial map) is not available, the SLAM process needs to be used to create it.

Inside the package lam_simulator there is a launch file (folder _localization_and_mapping/lam_simulator/launch/_). Run:

```
roslaunch lam_simulator gmapping_test_muraplast.launch
```


The launch file starts gmappping SLAM algorithm, the simulation arena with the MURAPLAST factory floorplan and RVIZ.
The robot can be moved by moving the mouse while holding the pointer over the robot, or by using the ROS teleoperation package which is used for the real robot:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

The mapped environment is visualized in RVIZ.
After you are sattisfied with the built map presented in RVIZ, save it and use it later for AMCL localization by starting the map saver:

```
rosrun map_server map_saver -f mapfile
```

# <a name="preparingmap">Preparing the map file to be used for localization and navigation</a>

As a result of SLAM you will obtain in the current folder where you called the map_saver command the mapfile.pgm and mapfile.yaml files.
This is an example:

```
image: mapfile.pgm
resolution: 0.068000
origin: [-25.024000, -25.024000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

To simulate this map in Stage you need to do the following. By default map_server and Stage have different reference global map coordinate systems. We choose to have the coordinate system at the lower left corner of the map. To have that first autocrop the mapfile.pgm so that borders are occupied, otherwise map_server and Stage will have different scales since Stage does the autocrop by itself. To do so open the mapfile.pgm in gimp, autocrop and put it in lam_simulator/worlds/elements folder for Stage world file and to yaml/bitmaps folder for map_server. Open the yaml file that is created by the map_saver, put origin to zero so that the origin will be the lower left corner and put it in the yaml folder. Do not forget to change the path to the mapfile.pgm (in this example bitmaps folder is added):

```
image: bitmaps/mapfile.pgm
resolution: 0.068000
origin: [0, 0, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Stage's world file should have defined size of the floorplan and pose of the floorplan which defines the coordinate frame. There is a parameter for a map resolution but it is ignored. Put the right size of the floorplan by calculating resolution*numpixels_col x resolution*numpixels_row and put the origin to half of it. 

Here is the example for the IML lab floorplan. The size of the map in pixels is 7057 x 2308 and the size of the pixel (resolution) is 0.007 m (the resolution is written in IMLlab.yaml file) so the Stage world file looks as follows:

```
floorplan( 
bitmap "elements/smartface_topologie_entwurf.png"
#map_resolution     0.068 
  pose [24.6995 8.078 0.000 0.000] 
  size [49.399 16.156 0.800] 
  name "IMLlab"
)
```
Maybe you need to set the threshold of the image to have all the obstacle visible also in rviz, as was the case with this png file, since Stage has different thresholds for occupied pixels.


# <a name="mapupdates">Map updates</a>

The map updates are the downsampled laser points to the fine resolution grid, e.g. 0.1 m cell size, which are not mapped in the initial map.
It is supposed that there is no moving obstacles in the environment, only the static ones that are not mapped in the initial map.
So if you move the green box in the Stage simulator the trail of the obstacle will be mapped and also remembered in the list of new obstacles.
By now, the list of new obstacles is never reset. This should be changed when taking into account dynamic obstacles.
The message that is returned is the simple array of coordinates (x,y) of all unmapped obstacles which are snapped to the fine resolution grid points:

NewObstacles.msg

	Header header # standard ROS header
	float64[] x		# x coordinate of the cell centre
	float64[] y		# y coordinate of the cell centre
	
The package _mapupdates_ creates the fine gridmap for keeping track of mapped and unmapped obstacles that needs to be run at the AGV computer. It is connected to the laser sensor on the AGV computer and it is important to set the right topic for the laser.
It is also important to set the robot id as args of the package, as is the example here:
```
<launch>

    <node name="mapup" pkg="mapupdates" type="mapup" output="screen" args="0" >
        <param name="cell_size" type="double" value="0.1" />
        <param name="scan_topic" value="/base_scan" />
        <param name="laser_inverted" type="bool" value="0" />
    </node>

</launch>
```
In this launch files you can set various parameters: **cell_size** defines the fine gridmap for calculating new obstacles; robot ID as an argument of packages _sending_and_perception_ and _mapupdates_ for having more robots (0,1,2,etc.), and **scan_topic** for defining the topic for the range data.
There is also a parameter **laser_inverted** to indicate if the laser is mounted upside down on the robot (use value 1), or normally (use value 0). The simplest check if laser is setup correctly is to see in rviz if laser readings are aligned with the png map.  
Right now only three robots are suported, i.e., with ID's 0, 1 and 2. If you change the ID of the robot to 1 the topic `/robot_1/newObstacles` will be created.
An example of the echo of the robot with ID 0 is:
```
$rostopic echo /robot_0/newObstacles
header: 
  seq: 438
  stamp: 
    secs: 53
    nsecs: 600000000
  frame_id: "map"
x: [1.55, 0.6500000000000001, 0.6500000000000001, 0.6500000000000001, 0.6500000000000001, 0.7500000000000001, 0.7500000000000001, 0.6500000000000001, 1.1500000000000001, 1.1500000000000001, 1.35, 1.35, 1.9500000000000002, 2.05, 2.05, 0.25, 0.55, 0.25, 0.25, 0.25, 0.25, 0.25, 0.35000000000000003, 0.7500000000000001, 0.6500000000000001, 0.7500000000000001, 0.7500000000000001, 0.7500000000000001, 0.7500000000000001, 0.7500000000000001, 1.1500000000000001, 1.7500000000000002, 2.05, 2.05, 2.05, 2.05, 2.05, 2.05, 2.25, 2.45, 2.55, 3.05, 3.05, 2.85, 2.75, 2.85, 2.85, 2.85, 3.05, 3.25, 3.25, 3.35, 3.45, 3.55, 3.65, 3.95, 3.95, 4.05, 4.15, 4.25, 4.55, 4.8500000000000005, 4.95, 4.95, 5.05, 5.15, 5.15, 5.25, 5.45, 6.3500000000000005, 6.45, 6.65, 6.75, 6.75, 6.95, 7.15, 7.25, 7.95, 8.15, 8.250000000000002, 8.250000000000002, 8.250000000000002, 8.450000000000001, 8.450000000000001, 8.250000000000002, 8.350000000000001, 8.850000000000001, 8.450000000000001, 8.450000000000001, 9.250000000000002, 9.350000000000001, 9.450000000000001, 9.55, 8.55, 8.65, 9.15, 9.250000000000002, 9.650000000000002, 9.750000000000002, 9.750000000000002, 9.750000000000002, 8.55, 8.55, 8.55, 8.55, 8.65, 8.55, 8.250000000000002, 8.05, 8.05, 8.15, 8.15, 8.250000000000002, 9.750000000000002, 9.750000000000002, 10.150000000000002, 9.950000000000001, 12.850000000000001, 12.750000000000002, 12.650000000000002, 10.05, 10.05, 10.05, 9.850000000000001, 8.65, 8.55, 8.450000000000001, 8.450000000000001, 8.350000000000001, 8.250000000000002, 0.0, 0.0, 8.05, 7.3500000000000005, 7.15, 7.15, 7.15, 7.05, 6.95, 6.8500000000000005, 6.45, 6.25, 5.75, 5.65, 5.55, 5.45, 5.3500000000000005, 5.25, 4.95, 4.55, 2.05, 1.9500000000000002, 2.35, 2.35, 2.75, 3.05, 3.05, 2.85, 2.85, 2.85, 2.75, 1.55, 1.55, 1.55, 1.4500000000000002, 0.25, 0.25, 1.05, 1.2500000000000002, 1.35, 1.55, 1.55]
y: [4.25, 4.05, 3.95, 3.85, 3.75, 3.75, 3.65, 3.55, 3.65, 3.55, 3.55, 3.45, 3.55, 3.55, 3.45, 2.95, 2.85, 2.65, 2.55, 2.45, 2.35, 2.25, 2.25, 2.25, 2.15, 2.15, 2.05, 1.9500000000000002, 1.85, 1.7500000000000002, 1.7500000000000002, 1.7500000000000002, 1.7500000000000002, 1.6500000000000001, 1.55, 1.4500000000000002, 1.35, 1.2500000000000002, 1.35, 1.35, 1.2500000000000002, 1.35, 1.2500000000000002, 0.9500000000000001, 0.6500000000000001, 0.6500000000000001, 0.55, 0.45, 0.45, 0.45, 0.35000000000000003, 0.35000000000000003, 0.35000000000000003, 0.35000000000000003, 0.35000000000000003, 0.25, 0.15000000000000002, 0.15000000000000002, 0.25, 0.15000000000000002, 0.35000000000000003, 1.05, 1.05, 0.9500000000000001, 1.05, 0.8500000000000001, 0.7500000000000001, 0.45, 0.35000000000000003, 0.45, 0.45, 0.45, 0.35000000000000003, 1.2500000000000002, 1.2500000000000002, 1.1500000000000001, 1.05, 0.45, 0.45, 0.35000000000000003, 0.45, 0.55, 0.35000000000000003, 0.55, 1.05, 1.05, 0.45, 1.05, 1.1500000000000001, 0.25, 0.25, 0.25, 0.25, 1.6500000000000001, 1.7500000000000002, 1.85, 1.85, 1.85, 1.7500000000000002, 1.85, 1.9500000000000002, 3.25, 3.35, 3.45, 3.55, 3.55, 3.85, 3.95, 4.55, 4.65, 4.65, 4.75, 4.8500000000000005, 5.05, 5.15, 5.55, 5.55, 6.8500000000000005, 6.95, 6.95, 6.8500000000000005, 7.15, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005, 7.45, 7.45, 7.45, 0.0, 0.0, 7.45, 7.15, 7.15, 7.25, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005, 5.65, 5.55, 5.55, 5.55, 5.55, 5.55, 5.65, 5.65, 5.65, 5.65, 6.95, 6.95, 6.55, 6.45, 6.25, 5.95, 5.8500000000000005, 5.8500000000000005, 5.75, 5.65, 5.55, 5.8500000000000005, 5.75, 5.65, 5.55, 5.75, 5.65, 5.25, 4.95, 4.75, 4.65, 4.55]
``` 

First start the AMCL localization in the known map and the simulator Stage in which laser data are simulated.
Then start the package mapupdates where new laser readings are compared to the cells of the gridmap. The package mapupdates converts the local laser readings to a global coordinate frame which can be visualized in rviz. This is used to test if the transformation is done correctly and marker points from topic /globalpoints_marker should be aligned in rviz over the simulated laser scan data.
And finaly, start maptogridmap to visualize the new obstacles and topology updates. The package maptogridmap is subscribed to topic /robot_0/newObstacles and checks if points belong to the free grid cell and changes its occupancy accordingly. Nodes and edges are updated too:

```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: roslaunch mapupdates startmapupdates.launch
terminal 3: roslaunch maptogridmap startmaptogridmap.launch
```

## <a name="writelis">Writing a simple listener explaining the maplistener package</a>

To read the topic in your own package you need to subscribe to it, include the header of the message, and write a message callback. The example is taken from maplistener/src/main.cpp.

* subscribe to a topic /robot_0/newObstacles: 
```
newobs_sub = nh_.subscribe("/robot_0/newObstacles",1,&VisualizationPublisherGML::newObstaclesCallback, this);
```
* include the header of the message in your header file or in the cpp where you are writting the message callback:
```
#include <mapupdates/NewObstacles.h>
```
* write a message callback for visualizing the marker topic /newobstacles_markerListener in rviz:
```
void VisualizationPublisherGML::newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
  glp.points.clear();
  geometry_msgs::Point p; 
	for (int i =0; i<msg->x.size(); i++){
		p.x=msg->x[i];
		p.y=msg->y[i];
		glp.points.push_back(p);
	}
}
```
* you also need to add in CMakeLists.txt of your package the line _mapupdates_ in find_package function otherwise the compiler will complain it can not find the header file:
```
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  roscpp
  tf
  maptogridmap
  mapupdates
)
```
* add depend _mapupdates_ in your package in package.xml:
```
  <build_depend>mapupdates</build_depend>
  <run_depend>mapupdates</run_depend>
```
Start the localization (AMCL) and laser readings (from simulator Stage) as explained in the Section [Map updates](#mapupdates):
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: roslaunch mapupdates startmapupdates.launch
```
* start maplistener to see the marker topic in rviz:
```
terminal 3: rosrun maplistener mapls
```

# <a name="examplesOCB">Examples</a>
## Sending pose with covariance and map updates to OCB (machine_2)

Use the following docker-compose.yml:
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
        image: mongo:3.6
        command: --nojournal   
```
Make a clean start of context broker: 
```
sudo docker-compose down
sudo docker-compose up
```
Check in firefox if http://OPIL_SERVER_IP:1026/v2/entities is blank (replace OPIL_SERVER_IP with the correct IP address).

Start the stage simulator and AMCL localization: 
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
```
Start sending pose with covariance:
```
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```
Start calculation of map updates:
```
terminal 3: roslaunch mapupdates startmapupdates.launch
```
If you want to send the topics through firos, you need to put in firos/config all json files from test/config_files/machine_2. Set the right IP addresses for your machine (server), OPIL server (contextbroker), and interface name (check with ifconfig) in firos/config/config.json and then run firos:
```
terminal 4: rosrun firos core.py
```

Now you can refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "robot_0" with topics "pose_channel" and "newObstacles".
Simply move the robot in stage by dragging it with the mouse and refresh the firefox to see the update of pose_channel and newObstacles.


## Receiving the pose with covariance and map updates through OCB (machine_1)

If you want to receive the topics through firos, you need to put in firos/config all json files from test/config_files/machine_1:

```
terminal 1: roscore
terminal 2: rosrun firos core.py
```
_mapupdates_ package needs to be on the machine_1 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo all ros topics:
```
rostopic echo /robot_0/pose_channel
rostopic echo /robot_0/newObstacles
```

* <a name="examplepose">Example output for the ICENT map - Pose with covariance</a>
<!--* Example output for the ICENT map - Pose with covariance-->
```
$rostopic echo /robot_0/pose_channel
header: 
  seq: 221
  stamp: 
    secs: 667
    nsecs: 200000000
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 5.89260851198
      y: 4.54273166596
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.000341716300675
      w: 0.999999941615
  covariance: [0.2280276789742004, 0.00121444362006784, 0.0, 0.0, 0.0, 0.0, 0.00121444362006784, 0.2095520253272909, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06568863093933444]
```



