In the following the list of ROS packages is given with the short explanation.
Afterwards, each package is explained with the followed examples.

# Overview of ROS packages

In src folder there are four main ROS packages:


## maptogridmap package

A ROS package for creating a gridmap with desired cell size from _map_server_ (PNG or PGM file) and merging the local map updates from an AGV into a gridmap, and a topology creation with annotations for the Task Planner in the form of a graph with nodes and edges. This package is located in the Central SP.

## mapupdates package

A ROS package for creating local map updates from the laser readings. This package is located in the Local SP on every AGV. Every AGV needs to start this package with the robot ID number set to 0, 1, etc., so that published topic has corresponding name as /robot_0/newObstacles, /robot_1/newObstacles, etc.

## maplistener package

A ROS package for testing subscribers to all created topics from mapupdates and maptogridmap packages and visualizing them in rviz.


## localization_and_mapping metapackage

A ROS metapackage containing packages for localization and mapping listed in the following: 


### localization_and_mapping packages

#### lam_simulator

ROS package which demonstrates localization and mapping in the Stage simulator. Relies on AMCL or gmapping algorithms started using the Stage simulator. Prepared demonstrations can also be used on the real robot.


#### sensing_and_perception

ROS package for publishing AGV's pose with covariance to be sent to Orion Context Broker through firos. This package is located in the Local SP.

#### andymark_driver

ROS drivers for omnidirectional Andymark platform. Teleoperation and control relies on the nodes provided by the _husky_ package.


#### husky

ROS package for interfacing with Clearpath Husky robot. It also includes nodes for teleoperation and control.


#### odometry_correction

ROS package which relies on _robot_pose_ekf_ to fuse robot odometry with IMU data to improve the odometry estimation.




# <a name="topology">Topology creation - maptogridmap package</a>
The topology is composed of nodes (vertices) and edges. It is single graph message composed of arrays of two ROS messages: Vertex and Edge.

Graph.msg

	Header header	# standard ROS header
	Vertex[] vertices # array of vertices
	Edge[] edges # array of edges

Vertex.msg

	float64 x	# x coordinate of the cell centre
	float64 y	# y coordinate of the cell centre
	float64 theta # orientation of the node used for annotations
	string name	# e.g. vertex_0 or annotation name
	string uuid	# unique id of a vertex created from its unique name
	geometry_msgs/Point[] footprint # four points of a squared footprint around a vertex of size cell_size
	
Edge.msg

	string uuid_src	# unique id of a source node of the edge
	string uuid_dest	# unique id of a destination node of the edge
	string name	# e.g. edge_0_1
	string uuid	# unique id of an edge created from its unique name
	
The creation of topology is started by launching the map_server package followed by launching the maptogridmap package which is explained in more details in the following text:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
You can also start a single launch file which combines these two launch files and also starts firos (this file is used in a docker container of Central SP):
```
roslaunch maptogridmap topology.launch
``` 
## Creation of nodes (vertices) in maptogridmap package
Nodes are all free cells of rectangular square size that does not contain any obstacle within it. The obstacles are read from the map PNG or PGM file loaded by calling the map_server node. There are three maps prepared in startmapserver.launch, where MURAPLAST floorplan is uncommented and IML lab and ICENT lab are commented out for the later usage.

```
<launch>
<!--MURAPLAST floorplan-->
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/floorplanMP.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>


<!--IML lab floorplan	
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/IMLlab.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
-->	
		
<!--ICENT lab floorplan
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/Andamapa.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
-->

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 

</launch>
```

The size of the cell is given by the parameter **cell_size** in startmaptogridmap.launch:
```
<launch>

    <node name="map2gm" pkg="maptogridmap" type="map2gm" >
        <param name="cell_size" type="double" value="2.0" />
        <param name="annotation_file" textfile="$(find maptogridmap)/launch/annotations.ini" />
    </node>

</launch>
```

In this example it is set to 2.0m, which is a default value of two diameters of the average AGV's footprint. Values that are presented in context broker are coordinates of the cell center (x,y) or coordinates of the manual annotation (loaded from a file), theta as an orientation of the annotated place in the map (default is 0), a name of the node in the form of "vertex_0" or rewritten by the name of the annotation, and the node's uuid (Universally unique identifier). The message that is sent through firos is maptogridmap/msg/Graph.msg.

## Creation of Annotations in maptogridmap package

Annotations can be loaded from file annotations.ini located in maptogridmap/launch folder, which is put as a parameter textfile inside the startmaptogridmap.launch file. In this example the first three annotations were used in Zagreb demo in Task planner, and P1 is put as an additional example for the IML map:
```
[loadingArea]
# coordinates
point_x = 5.75
point_y = 2.51
theta = 0
distance = 1.6

[unloadingArea]
# coordinates
point_x = 8.4
point_y = 2.51
theta = 0 
distance = 1.4

[waitingArea]
# coordinates
point_x = 6.9
point_y = 6.3
theta = 90
distance = 1.6

[P1]
# coordinates
point_x = 17.96
point_y = 6.57
theta = -90
distance = 1.8
```
The annotations are saved under the variable of type maptogridmap::Annotations inside of the maptogridmap package. All values must be in meters and degrees. From these values it is calculated where the AGV needs to be placed in front of the annotation according to the distance from the annotation and the orientation theta. It changes the values of the computed nodes from gridmap cells so that TP can use this nodes as goals.

These four annotations change the coordinates of the cell centre of the grid map (but only free cells) and also change the name to the annotation name, e.g., loadingArea, unloadingArea, etc. The result can be seen in [topic /map/graph.](#exampleannot)


## Creation of Edges in maptogridmap package
Edges are pairs of neighbor nodes. Neighbors are defined between two nodes which have their centres' coordinates distanced for _cell_size_. The edges are bidirectional, meaning two neighbor nodes n and m forms the edge (n,m) and (m,n) which are identical.
Values of Edges are the source node's uuid, named as _uuid_src_, the destination node's uuid, named as _uuid_dest_, the name of the edge in the form of "edge_0_1" meaning that two nodes with names "vertex_0" and "vertex_1" are connected with the edge, and the edge's uuid. 
The message that is sent through firos is maptogridmap/msg/Graph.msg

## <a name="writelis">Writing a simple listener explaining the maplistener package</a>

To read the topic in your own package you need to subscribe to it, include the header of the message, and write a message callback. The example is taken from maplistener/src/main.cpp.

* subscribe to a topics /map/graph 
```
 	graph_sub = nh_.subscribe("map/graph",1,&VisualizationPublisherGML::graphCallback, this);

```
* include the header of the message in your header file or in the cpp where you are writing the message callback:
```
#include <maptogridmap/Graph.h>
```
* write a message callback for Graph message, which draws the footprint of the robot around each vertex
```
void VisualizationPublisherGML::graphCallback(const maptogridmap::GraphConstPtr& gmMsg)
{
  footprint.points.clear();
  geometry_msgs::Point p; 
	for (int i=0; i<gmMsg->vertices.size(); i++){
		for (int d=0; d<4;d++){
			p.x=gmMsg->vertices[i].footprint[d].x;
			p.y=gmMsg->vertices[i].footprint[d].y;
			footprint.points.push_back(p);
			if (d<3){
			p.x=gmMsg->vertices[i].footprint[d+1].x;
			p.y=gmMsg->vertices[i].footprint[d+1].y;
			footprint.points.push_back(p);
			}
		}
		p.x=gmMsg->vertices[i].footprint[0].x;
		p.y=gmMsg->vertices[i].footprint[0].y;
		footprint.points.push_back(p);
	}
}
```
* add in CMakeLists.txt of your package the line _maptogridmap_ in find_package function otherwise the compiler will complain it can not find the header file:
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
* add depend _maptogridmap_ in your package in package.xml:
```
  <build_depend>maptogridmap</build_depend>
  <run_depend>maptogridmap</run_depend>
```
You can test how subscribed topics are visualized in rviz by typing:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
terminal 3: rosrun maplistener mapls
```
The nodes are visualized with the marker topic /nodes_markerListener, while the edges are visualized with the marker topic /edges_markerListener.
The package maplistener also subscribes to map updates for which you first need to have localization (AMCL) and laser readings (from simulator Stage) as explained in the Section [Map updates](#mapupdates):
```
terminal 4: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 5: roslaunch mapupdates startmapupdates.launch
```

# <a name="poswithcov">Pose with covariance</a>

A standard ROS message is used for sending the pose of the AGV:
	
	geometry_msgs.msg.PoseWithCovarianceStamped
	
To be able to send this message, two modules needs to be started: the AMCL package and the sensing_and_perception package. The AMCL package provides localization of the AGV inside a given map using the odometry and laser sensors. The sensing_and_perception package combines the covariance calculated by the AMCL package and the the global pose in the map frame as a result of a combination of odometry and AMCL localization with the laser.
Since this topic will be send through firos to OCB, the unique robot id needs to be set in the lauch file through _args_, e.g. 0, 1, 2, ... In this example launch file send_posewithcovariance.launch you can see that the id of the robot is set to 0:
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

#SLAM

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

To simulate this map in Stage you need to do the following. By default map_server and Stage have different reference global map coordinate systems. We choose to have the coordinate system at the lower left corner of the map. To have that first autocrop the mapfile.pgm so that borders are occupied, otherwise map_server and Stage will have different scales since Stage does the autocrop by itself. To do so open the mapfile.pgm in gimp, autocrop and put it in lam_simulator/worlds/elements folder for Stage world file and to yaml/bitmaps folder for map_server. Open the yaml file that is created by the map_saver, put origin to zero so that the origin will be the lower left corner and put it in the yaml folder. Do not forget to change the path to the mapfile.pgm (in this example bitmaps folder is added).

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
    </node>

</launch>
```
The topic that is returned contains the robot ID, e.g. if set to 0 the topic will be /robot_0/newObstacles and the example of the echo of it is:
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
And finaly, start maptogridmap to visualize the new obstacles and topology updates. The package maptogridmap is subscribed to topic /robot_0/newObstacles and checks if points belong to the free grid cell and changes its occupancy accordingly. Nodes and edges are updated too. 

```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: roslaunch mapupdates startmapupdates.launch
terminal 3: roslaunch maptogridmap startmaptogridmap.launch
```
To read the topic in your own package you need to subscribe to it, include the header of the message, and write a message callback (as is the case in the maptogridmap package).

* subscribe to a topic /robot_0/newObstacles
```
  ros::Subscriber gmu_sub = nh.subscribe("/robot_0/newObstacles",1,newObstaclesCallback);

```
* include the header of the message in your header file or in the cpp where you are writting the message callback:
```
#include <mapupdates/NewObstacles.h>
```
* write a message callback (in this example we just rewrite the global variable obstacles that is used in the main program):
```
mapupdates::NewObstacles obstacles;	#global variable
void newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
	obstacles.x.clear();
	obstacles.y.clear();
	for (int i =0; i<msg->x.size(); i++){
		obstacles.x.push_back(msg->x[i]);
		obstacles.y.push_back(msg->y[i]);
	}
}
```
* maybe you also need to add in CMakeLists.txt of your package the line _mapupdates_ in find_package function otherwise the compiler will complain ti can not find the header file:
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

Another example of subscribing to a topic /robot_0/newObstacles is in _maplistener_ package:
```
newobs_sub = nh_.subscribe("/robot_0/newObstacles",1,&VisualizationPublisherGML::newObstaclesCallback, this);
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
* start maplistener to see the marker topic in rviz:
```
terminal 4: rosrun maplistener mapls
```

# Examples
## Testing if ROS topics for Nodes and Edges are sent to Orion Context Broker:
TODO: change to graph

Make a clean start of context broker (use the docker-compose.yml in test/docker_compose_files/Central_SP_docker):
```
sudo docker-compose down
sudo docker-compose up
```
Check in firefox if http://OPIL_SERVER_IP:1026/v2/entities is blank (replace OPIL_SERVER_IP with the correct IP address).

On machine 1 start:

* Uncomment the map you want to use (MP, IML or Anda) and comment the rest of maps in startmapserver.launch.
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
If you want to send the topics through firos, you need to put in firos/config all json files from test/config_files/machine_1:
```
terminal 3: rosrun firos core.py
```

Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities. There should be under id "map" topics "nodes" and "edges" with their values.


## Testing sending pose with covariance on machine_1
Just start after previous three terminals. If you want to test only sending pose with covariance, repeat the commands in section [Pose with covariance](#poswithcov).
Start the simulation Stage with the amcl localization depending on the map you are using in startmapserver.launch. For example, if you are using MP map use amcl_test_muraplast.launch; if you are using IML map use IMLamcltest.launch; otherwise, if you are using ICENT map use the following:
```
terminal 4: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
```
This starts stage simulator and amcl localization. 
```
terminal 5: roslaunch sensing_and_perception send_posewithcovariance.launch 
```
Now you can refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "robot_0" with topic "pose_channel".
Simply move the robot in stage by dragging it with the mouse and refresh the firefox to see the update of pose_channel.


## Testing if topics for Graph and PoseWithCovariance are received on machine_2 through firos

If you want to receive the topics through firos, you need to put in firos/config all json files from test/config_files/machine_2:

```
terminal 1: roscore
terminal 2: rosrun firos core.py
```
_maptogridmap_ package needs to be on the machine_2 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo all ros topics:
```
rostopic echo /map/graph
rostopic echo /robot_0/pose_channel
```
* <a name="exampleannot">Example output for the ICENT map - graph (vertices and edges) with loaded annotations</a>
<!--* Example output for the ICENT map - Nodes-->

```
$rostopic echo /map/graph
---
header: 
  seq: 87
  stamp: 
    secs: 879
    nsecs:         0
  frame_id: "map"
vertices: 
  - 
    x: 4.15
    y: 2.51
    theta: 0.0
    name: "loadingArea"
    uuid: "5204f8fa-34fb-5cad-b85e-654f19d4ebf5"
    footprint: 
      - 
        x: 3.35
        y: 1.71
        z: 0.0
      - 
        x: 4.95
        y: 1.71
        z: 0.0
      - 
        x: 4.95
        y: 3.31
        z: 0.0
      - 
        x: 3.35
        y: 3.31
        z: 0.0
  - 
    x: 4.0
    y: 4.0
    theta: 0.0
    name: "vertex_10"
    uuid: "3dcd4aec-1a4f-5156-a517-17cdcf55ad47"
    footprint: 
      - 
        x: 3.2
        y: 3.2
        z: 0.0
      - 
        x: 4.8
        y: 3.2
        z: 0.0
      - 
        x: 4.8
        y: 4.8
        z: 0.0
      - 
        x: 3.2
        y: 4.8
        z: 0.0
  - 
    x: 5.6
    y: 2.4
    theta: 0.0
    name: "vertex_13"
    uuid: "45a28b12-3dc3-5dc3-a687-14fc7fa3be89"
    footprint: 
      - 
        x: 4.8
        y: 1.6
        z: 0.0
      - 
        x: 6.4
        y: 1.6
        z: 0.0
      - 
        x: 6.4
        y: 3.2
        z: 0.0
      - 
        x: 4.8
        y: 3.2
        z: 0.0
  - 
    x: 5.6
    y: 4.0
    theta: 0.0
    name: "vertex_14"
    uuid: "439be9e5-a52a-5cea-88e2-ab6150ffe578"
    footprint: 
      - 
        x: 4.8
        y: 3.2
        z: 0.0
      - 
        x: 6.4
        y: 3.2
        z: 0.0
      - 
        x: 6.4
        y: 4.8
        z: 0.0
      - 
        x: 4.8
        y: 4.8
        z: 0.0
  - 
    x: 7.0
    y: 2.51
    theta: 0.0
    name: "unloadingArea"
    uuid: "9ba5016b-e219-513f-af73-ebce59f92a9b"
    footprint: 
      - 
        x: 6.2
        y: 1.71
        z: 0.0
      - 
        x: 7.8
        y: 1.71
        z: 0.0
      - 
        x: 7.8
        y: 3.31
        z: 0.0
      - 
        x: 6.2
        y: 3.31
        z: 0.0
  - 
    x: 6.9
    y: 4.7
    theta: 90.0
    name: "waitingArea"
    uuid: "4109424b-20d0-5707-869c-dc62fea9a658"
    footprint: 
      - 
        x: 6.1
        y: 3.9
        z: 0.0
      - 
        x: 7.7
        y: 3.9
        z: 0.0
      - 
        x: 7.7
        y: 5.5
        z: 0.0
      - 
        x: 6.1
        y: 5.5
        z: 0.0
  - 
    x: 12.0
    y: 0.8
    theta: 0.0
    name: "vertex_28"
    uuid: "8c4754ff-8571-5467-9da8-3da0d3ba8caa"
    footprint: 
      - 
        x: 11.2
        y: 0.0
        z: 0.0
      - 
        x: 12.8
        y: 0.0
        z: 0.0
      - 
        x: 12.8
        y: 1.6
        z: 0.0
      - 
        x: 11.2
        y: 1.6
        z: 0.0
  - 
    x: 12.0
    y: 2.4
    theta: 0.0
    name: "vertex_29"
    uuid: "d30fd462-a84d-5940-b533-3f486e0b8561"
    footprint: 
      - 
        x: 11.2
        y: 1.6
        z: 0.0
      - 
        x: 12.8
        y: 1.6
        z: 0.0
      - 
        x: 12.8
        y: 3.2
        z: 0.0
      - 
        x: 11.2
        y: 3.2
        z: 0.0
  - 
    x: 12.0
    y: 4.0
    theta: 0.0
    name: "vertex_30"
    uuid: "7d59bcca-aaac-51fc-b3da-3006bae4e835"
    footprint: 
      - 
        x: 11.2
        y: 3.2
        z: 0.0
      - 
        x: 12.8
        y: 3.2
        z: 0.0
      - 
        x: 12.8
        y: 4.8
        z: 0.0
      - 
        x: 11.2
        y: 4.8
        z: 0.0
edges: 
  - 
    uuid_src: "5204f8fa-34fb-5cad-b85e-654f19d4ebf5"
    uuid_dest: "45a28b12-3dc3-5dc3-a687-14fc7fa3be89"
    name: "edge_9_13"
    uuid: "35dbf846-4fbd-5ad7-a69d-7dc6fcf8fec3"
  - 
    uuid_src: "5204f8fa-34fb-5cad-b85e-654f19d4ebf5"
    uuid_dest: "3dcd4aec-1a4f-5156-a517-17cdcf55ad47"
    name: "edge_9_10"
    uuid: "57f9dc88-47fb-53f0-b119-b4ffd262ce21"
  - 
    uuid_src: "3dcd4aec-1a4f-5156-a517-17cdcf55ad47"
    uuid_dest: "439be9e5-a52a-5cea-88e2-ab6150ffe578"
    name: "edge_10_14"
    uuid: "81d06871-ae9a-50a6-95ff-8e6a761eb004"
  - 
    uuid_src: "45a28b12-3dc3-5dc3-a687-14fc7fa3be89"
    uuid_dest: "9ba5016b-e219-513f-af73-ebce59f92a9b"
    name: "edge_13_17"
    uuid: "8d5b125d-2a90-54e9-8bf9-45ad9406de5b"
  - 
    uuid_src: "45a28b12-3dc3-5dc3-a687-14fc7fa3be89"
    uuid_dest: "439be9e5-a52a-5cea-88e2-ab6150ffe578"
    name: "edge_13_14"
    uuid: "aeb7a192-addc-55c1-9505-db6a05bd6069"
  - 
    uuid_src: "439be9e5-a52a-5cea-88e2-ab6150ffe578"
    uuid_dest: "4109424b-20d0-5707-869c-dc62fea9a658"
    name: "edge_14_18"
    uuid: "e94216cc-7db7-5eed-9597-a6ae3299e0ea"
  - 
    uuid_src: "9ba5016b-e219-513f-af73-ebce59f92a9b"
    uuid_dest: "4109424b-20d0-5707-869c-dc62fea9a658"
    name: "edge_17_18"
    uuid: "1aa38a37-e15f-54e6-a595-2874c90c2a56"
  - 
    uuid_src: "8c4754ff-8571-5467-9da8-3da0d3ba8caa"
    uuid_dest: "d30fd462-a84d-5940-b533-3f486e0b8561"
    name: "edge_28_29"
    uuid: "cea0b000-a71d-5b4e-a892-0ad6d34bc887"
  - 
    uuid_src: "d30fd462-a84d-5940-b533-3f486e0b8561"
    uuid_dest: "7d59bcca-aaac-51fc-b3da-3006bae4e835"
    name: "edge_29_30"
    uuid: "fc169931-93fe-5198-bef0-a2e512ad579f"
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




