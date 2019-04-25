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
## Creation of Nodes in maptogridmap package
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

In this example it is set to 2.0m since the floorplan is quite big. ICENT lab is much smaller so 1.2m cell size gives better results. Values that are presented in context broker are coordinates of the cell center (x,y) or coordinates of the manual annotation (loaded from a file), theta as an orientation of the annotated place in the map (default is 0), a name of the node in the form of "vertex_0" or rewritten by the name of the annotation, and the node's uuid (Universally unique identifier). The message that is sent through firos is maptogridmap/msg/Graph.msg.

## Creation of Annotations in maptogridmap package

Annotations can be loaded from file annotations.ini located in maptogridmap/launch folder, which is put as a parameter textfile inside the startmaptogridmap.launch file. In this example the first three annotations were used in Zagreb demo in Task planner, and P1 is put as an additional example for the IML map:
```
[loadingArea]
# coordinates
point_x = 3.75
point_y = 2.51
theta = 0
distance = 1.6

[unloadingArea]
# coordinates
point_x = 6.4
point_y = 2.51
theta = 0 
distance = 1.4

[waitingArea]
# coordinates
point_x = 6.9
point_y = 4.3
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
```
x[]
  x[0]: 3.75
  x[1]: 6.4
  x[2]: 6.9
  x[3]: 17.96
y[]
  y[0]: 2.51
  y[1]: 2.51
  y[2]: 4.3
  y[3]: 6.57
theta[]
  theta[0]: 0
  theta[1]: 0
  theta[2]: 90
  theta[3]: -90
distance[]
  distance[0]: 1.6
  distance[1]: 1.4
  distance[2]: 1.6
  distance[3]: 1.8
name[]
  name[0]: loadingArea
  name[1]: unloadingArea
  name[2]: waitingArea
  name[3]: P1
```
These four annotations change the coordinates of the cell centre of the grid map (but only free cells) and also change the name to the annotation name, e.g., loadingArea, unloadingArea, etc. The result can be seen in [topic /map/nodes.](#exampleannot)


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
* write a message callback for nodes and edges (TODO: only partially changed for the new Graph.msg)
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


## Testing if topics for Nodes, Edges, and PoseWithCovariance are received on machine_2 through firos
TODO: change to graph

If you want to receive the topics through firos, you need to put in firos/config all json files from test/config_files/machine_2:

```
terminal 1: roscore
terminal 2: rosrun firos core.py
```
_maptogridmap_ package needs to be on the machine_2 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo all ros topics:
```
rostopic echo /map/nodes
rostopic echo /map/edges
rostopic echo /robot_0/pose_channel
```
* <a name="exampleannot">Example output for the ICENT map - Nodes with loaded annotations</a>
<!--* Example output for the ICENT map - Nodes-->

```
$rostopic echo /map/nodes
header: 
  seq: 104
  stamp: 
    secs: 65
    nsecs: 300000000
  frame_id: "map"
info: 
  map_load_time: 
    secs: 0
    nsecs:         0
  resolution: 1.0
  width: 14
  height: 8
  origin: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
x: [1.5, 2.5, 2.5, 2.9, 3.5, 3.75, 3.5, 3.5, 3.5, 3.5, 4.5, 4.5, 4.5, 4.5, 4.5, 5.5, 5.5, 5.5, 5.5, 6.4, 6.5, 6.9, 7.5, 7.5, 7.5, 7.5, 7.5, 8.5, 8.5, 9.5, 10.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 12.5, 12.5, 12.5, 12.5, 12.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5]
y: [2.5, 2.5, 3.5, 4.6, 1.5, 2.51, 3.5, 4.5, 5.5, 6.5, 1.5, 2.5, 3.5, 4.5, 6.5, 1.5, 2.5, 3.5, 4.5, 2.51, 3.5, 4.3, 2.5, 3.5, 4.5, 5.5, 6.5, 5.5, 6.5, 6.5, 1.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 0.5, 1.5, 2.5, 3.5, 4.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 7.5]
theta: [0.0, 0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
name: [vertex_10, vertex_18, vertex_19, P1, vertex_25, loadingArea, vertex_27, vertex_28,
  vertex_29, vertex_30, vertex_33, vertex_34, vertex_35, vertex_36, vertex_38, vertex_41,
  vertex_42, vertex_43, vertex_44, unloadingArea, vertex_51, waitingArea, vertex_58,
  vertex_59, vertex_60, vertex_61, vertex_62, vertex_69, vertex_70, vertex_78, vertex_81,
  vertex_88, vertex_89, vertex_90, vertex_91, vertex_92, vertex_93, vertex_94, vertex_96,
  vertex_97, vertex_98, vertex_99, vertex_100, vertex_104, vertex_105, vertex_106,
  vertex_107, vertex_108, vertex_109, vertex_111]
uuid: [57c319ef-5830-4035-bfd4-bb4852bf7f3e, c83f8271-c67c-48df-b654-b69930846891, 15818a4b-6749-4d8f-a550-15a6291b9d28,
  70536798-2680-43f8-a415-64d70dd249dd, 4931ab8c-9f2f-4913-8768-98edbd75f543, 6cf2954e-dafa-45f1-ba21-3a4f64909153,
  1173257d-2ece-4dd1-98ba-9d762201b592, c33bc043-55da-4689-aec4-d82ec4a31197, 89f004df-c2fd-43cf-bb61-6d7bf83d50ea,
  757ac47e-33b3-4acf-ad85-81c431667806, 0ca96b41-e949-426b-afab-c183e7be1ff2, 9b369197-9275-4db4-8b8b-5ebabaa7b833,
  3e29ca55-bb7d-4b91-a9c9-4f5cbfff29cb, 275b3195-152f-4659-8b96-9e4683a1af91, 172086aa-5067-4f74-9a65-318872d9f9e6,
  97fddd32-361e-4c8c-b1a6-c99eab4da583, 9b2a0e09-14df-4667-858a-4395dcc85170, 123f665a-5cab-41bf-a872-bf998f647f5c,
  d27810f6-689a-47b6-a5c9-d5462baa9dd9, 2c2be0b8-a750-499f-89bb-83c178a0809b, d4c709ab-fa4a-4b87-9f43-269af26f436a,
  6416975f-747f-439e-9bea-987c7e5ba9cc, 4bd3cc9d-c3c1-468a-934f-430a2863fbc7, 80c3750b-536a-464f-bd79-f2356ef68f58,
  a3d23430-38ed-42c4-8d9e-2d638a1697bd, 01fd7093-4716-45d2-8e8b-6ddeb04f5caf, 715e4b36-134b-4c0d-83ee-398585d2c296,
  d98e4e0f-7be1-40c4-b355-01d8edee2919, 0a660726-83cb-4b09-99f3-57df923b46a2, 2f543486-e47d-48e7-89bd-08743d509c55,
  06619538-37af-47ec-a642-657642f0eca4, 192b0899-c1d4-4458-a9b8-c4da5fc51ba6, dbbc46e7-93f7-492a-b758-d92a398f4e65,
  3d187492-0862-44dd-b28e-2d6e53b7ede4, a5aa74ab-9949-4a69-951b-1192eb9f04f3, a0178b30-50ef-4d69-9b32-f4e6a4bd8a11,
  51190db8-d13b-4838-8a09-452d9f46c030, e62dd631-99aa-40eb-8001-11b720823bb1, eff5f5a0-54db-4f38-a35e-75ef77abe4b9,
  3f1d9f14-9c96-421e-ab47-169007dd4fae, d8c89d2c-9a69-4dbf-a113-9c9d0d5a9bd4, d3a54686-f687-4791-845f-71111f390770,
  36a0a772-fe64-46aa-8f12-252dba151445, d0b5b5fb-5242-4bbc-b893-ced7727b334c, 2883a9ef-d003-4446-b634-929e055b8465,
  0625388d-b9c9-4c28-8b73-bd3637f686f4, fd27eca8-6744-4a75-be29-e9a0adf189c0, 28023451-3b2c-4b4e-9d07-171d3eb2acc4,
  ff9fa735-260f-4ec3-b2a4-9896ef14fc27, e014d0b0-1130-4e02-b402-1aab060a9a10]
```

* Example output for the ICENT map - Edges

```
$rostopic echo /map/edges
header: 
  seq: 4893
  stamp: 
    secs: 544
    nsecs: 200000000
  frame_id: "map"
uuid_src: [57c319ef-5830-4035-bfd4-bb4852bf7f3e, c83f8271-c67c-48df-b654-b69930846891, c83f8271-c67c-48df-b654-b69930846891,
  15818a4b-6749-4d8f-a550-15a6291b9d28, 15818a4b-6749-4d8f-a550-15a6291b9d28, 70536798-2680-43f8-a415-64d70dd249dd,
  4931ab8c-9f2f-4913-8768-98edbd75f543, 4931ab8c-9f2f-4913-8768-98edbd75f543, 6cf2954e-dafa-45f1-ba21-3a4f64909153,
  6cf2954e-dafa-45f1-ba21-3a4f64909153, 1173257d-2ece-4dd1-98ba-9d762201b592, 1173257d-2ece-4dd1-98ba-9d762201b592,
  c33bc043-55da-4689-aec4-d82ec4a31197, c33bc043-55da-4689-aec4-d82ec4a31197, 89f004df-c2fd-43cf-bb61-6d7bf83d50ea,
  757ac47e-33b3-4acf-ad85-81c431667806, 0ca96b41-e949-426b-afab-c183e7be1ff2, 0ca96b41-e949-426b-afab-c183e7be1ff2,
  9b369197-9275-4db4-8b8b-5ebabaa7b833, 9b369197-9275-4db4-8b8b-5ebabaa7b833, 3e29ca55-bb7d-4b91-a9c9-4f5cbfff29cb,
  3e29ca55-bb7d-4b91-a9c9-4f5cbfff29cb, 275b3195-152f-4659-8b96-9e4683a1af91, 97fddd32-361e-4c8c-b1a6-c99eab4da583,
  9b2a0e09-14df-4667-858a-4395dcc85170, 9b2a0e09-14df-4667-858a-4395dcc85170, 123f665a-5cab-41bf-a872-bf998f647f5c,
  123f665a-5cab-41bf-a872-bf998f647f5c, d27810f6-689a-47b6-a5c9-d5462baa9dd9, 2c2be0b8-a750-499f-89bb-83c178a0809b,
  2c2be0b8-a750-499f-89bb-83c178a0809b, d4c709ab-fa4a-4b87-9f43-269af26f436a, d4c709ab-fa4a-4b87-9f43-269af26f436a,
  6416975f-747f-439e-9bea-987c7e5ba9cc, 4bd3cc9d-c3c1-468a-934f-430a2863fbc7, 80c3750b-536a-464f-bd79-f2356ef68f58,
  a3d23430-38ed-42c4-8d9e-2d638a1697bd, 01fd7093-4716-45d2-8e8b-6ddeb04f5caf, 01fd7093-4716-45d2-8e8b-6ddeb04f5caf,
  715e4b36-134b-4c0d-83ee-398585d2c296, d98e4e0f-7be1-40c4-b355-01d8edee2919, 0a660726-83cb-4b09-99f3-57df923b46a2,
  06619538-37af-47ec-a642-657642f0eca4, 192b0899-c1d4-4458-a9b8-c4da5fc51ba6, dbbc46e7-93f7-492a-b758-d92a398f4e65,
  dbbc46e7-93f7-492a-b758-d92a398f4e65, 3d187492-0862-44dd-b28e-2d6e53b7ede4, 3d187492-0862-44dd-b28e-2d6e53b7ede4,
  a5aa74ab-9949-4a69-951b-1192eb9f04f3, a5aa74ab-9949-4a69-951b-1192eb9f04f3, a0178b30-50ef-4d69-9b32-f4e6a4bd8a11,
  a0178b30-50ef-4d69-9b32-f4e6a4bd8a11, 51190db8-d13b-4838-8a09-452d9f46c030, eff5f5a0-54db-4f38-a35e-75ef77abe4b9,
  3f1d9f14-9c96-421e-ab47-169007dd4fae, 3f1d9f14-9c96-421e-ab47-169007dd4fae, d8c89d2c-9a69-4dbf-a113-9c9d0d5a9bd4,
  d8c89d2c-9a69-4dbf-a113-9c9d0d5a9bd4, d3a54686-f687-4791-845f-71111f390770, d3a54686-f687-4791-845f-71111f390770,
  36a0a772-fe64-46aa-8f12-252dba151445, d0b5b5fb-5242-4bbc-b893-ced7727b334c, 2883a9ef-d003-4446-b634-929e055b8465,
  0625388d-b9c9-4c28-8b73-bd3637f686f4, fd27eca8-6744-4a75-be29-e9a0adf189c0, 28023451-3b2c-4b4e-9d07-171d3eb2acc4]
uuid_dest: [c83f8271-c67c-48df-b654-b69930846891, 6cf2954e-dafa-45f1-ba21-3a4f64909153, 15818a4b-6749-4d8f-a550-15a6291b9d28,
  1173257d-2ece-4dd1-98ba-9d762201b592, 70536798-2680-43f8-a415-64d70dd249dd, c33bc043-55da-4689-aec4-d82ec4a31197,
  0ca96b41-e949-426b-afab-c183e7be1ff2, 6cf2954e-dafa-45f1-ba21-3a4f64909153, 9b369197-9275-4db4-8b8b-5ebabaa7b833,
  1173257d-2ece-4dd1-98ba-9d762201b592, 3e29ca55-bb7d-4b91-a9c9-4f5cbfff29cb, c33bc043-55da-4689-aec4-d82ec4a31197,
  275b3195-152f-4659-8b96-9e4683a1af91, 89f004df-c2fd-43cf-bb61-6d7bf83d50ea, 757ac47e-33b3-4acf-ad85-81c431667806,
  172086aa-5067-4f74-9a65-318872d9f9e6, 97fddd32-361e-4c8c-b1a6-c99eab4da583, 9b369197-9275-4db4-8b8b-5ebabaa7b833,
  9b2a0e09-14df-4667-858a-4395dcc85170, 3e29ca55-bb7d-4b91-a9c9-4f5cbfff29cb, 123f665a-5cab-41bf-a872-bf998f647f5c,
  275b3195-152f-4659-8b96-9e4683a1af91, d27810f6-689a-47b6-a5c9-d5462baa9dd9, 9b2a0e09-14df-4667-858a-4395dcc85170,
  2c2be0b8-a750-499f-89bb-83c178a0809b, 123f665a-5cab-41bf-a872-bf998f647f5c, d4c709ab-fa4a-4b87-9f43-269af26f436a,
  d27810f6-689a-47b6-a5c9-d5462baa9dd9, 6416975f-747f-439e-9bea-987c7e5ba9cc, 4bd3cc9d-c3c1-468a-934f-430a2863fbc7,
  d4c709ab-fa4a-4b87-9f43-269af26f436a, 80c3750b-536a-464f-bd79-f2356ef68f58, 6416975f-747f-439e-9bea-987c7e5ba9cc,
  a3d23430-38ed-42c4-8d9e-2d638a1697bd, 80c3750b-536a-464f-bd79-f2356ef68f58, a3d23430-38ed-42c4-8d9e-2d638a1697bd,
  01fd7093-4716-45d2-8e8b-6ddeb04f5caf, d98e4e0f-7be1-40c4-b355-01d8edee2919, 715e4b36-134b-4c0d-83ee-398585d2c296,
  0a660726-83cb-4b09-99f3-57df923b46a2, 0a660726-83cb-4b09-99f3-57df923b46a2, 2f543486-e47d-48e7-89bd-08743d509c55,
  dbbc46e7-93f7-492a-b758-d92a398f4e65, dbbc46e7-93f7-492a-b758-d92a398f4e65, 3f1d9f14-9c96-421e-ab47-169007dd4fae,
  3d187492-0862-44dd-b28e-2d6e53b7ede4, d8c89d2c-9a69-4dbf-a113-9c9d0d5a9bd4, a5aa74ab-9949-4a69-951b-1192eb9f04f3,
  d3a54686-f687-4791-845f-71111f390770, a0178b30-50ef-4d69-9b32-f4e6a4bd8a11, 36a0a772-fe64-46aa-8f12-252dba151445,
  51190db8-d13b-4838-8a09-452d9f46c030, e62dd631-99aa-40eb-8001-11b720823bb1, 3f1d9f14-9c96-421e-ab47-169007dd4fae,
  2883a9ef-d003-4446-b634-929e055b8465, d8c89d2c-9a69-4dbf-a113-9c9d0d5a9bd4, 0625388d-b9c9-4c28-8b73-bd3637f686f4,
  d3a54686-f687-4791-845f-71111f390770, fd27eca8-6744-4a75-be29-e9a0adf189c0, 36a0a772-fe64-46aa-8f12-252dba151445,
  28023451-3b2c-4b4e-9d07-171d3eb2acc4, 2883a9ef-d003-4446-b634-929e055b8465, 0625388d-b9c9-4c28-8b73-bd3637f686f4,
  fd27eca8-6744-4a75-be29-e9a0adf189c0, 28023451-3b2c-4b4e-9d07-171d3eb2acc4, ff9fa735-260f-4ec3-b2a4-9896ef14fc27]
name: [edge_10_18, edge_18_26, edge_18_19, edge_19_27, edge_19_20, edge_20_28, edge_25_33,
  edge_25_26, edge_26_34, edge_26_27, edge_27_35, edge_27_28, edge_28_36, edge_28_29,
  edge_29_30, edge_30_38, edge_33_41, edge_33_34, edge_34_42, edge_34_35, edge_35_43,
  edge_35_36, edge_36_44, edge_41_42, edge_42_50, edge_42_43, edge_43_51, edge_43_44,
  edge_44_52, edge_50_58, edge_50_51, edge_51_59, edge_51_52, edge_52_60, edge_58_59,
  edge_59_60, edge_60_61, edge_61_69, edge_61_62, edge_62_70, edge_69_70, edge_70_78,
  edge_81_89, edge_88_89, edge_89_97, edge_89_90, edge_90_98, edge_90_91, edge_91_99,
  edge_91_92, edge_92_100, edge_92_93, edge_93_94, edge_96_97, edge_97_105, edge_97_98,
  edge_98_106, edge_98_99, edge_99_107, edge_99_100, edge_100_108, edge_104_105, edge_105_106,
  edge_106_107, edge_107_108, edge_108_109]
uuid: [54d7fef2-4b84-4af3-817e-1372eb7ce426, c85274d8-e8a4-4381-998d-5d2b29cdac07, 7fd651fa-5658-4dfb-b077-e43cac0e0947,
  dd35b234-3d93-473e-a3e1-e36c2a548ab0, f2b0062c-2e84-4e3b-a012-4a3d34c6fcec, 25d00572-22a1-4b78-b708-6ad77e1ad571,
  2faeb79f-f610-4094-abe5-ec3c497adac7, dab00789-1795-4e4a-a501-5e2f6a4a52dd, d59148f3-045f-4e46-b1bf-9a2b562e14f2,
  c5fc202f-ad70-4ee0-97f4-2497ec3a0e31, f438e348-3107-4d89-ad80-c3191537ac85, a970bafc-0136-4f9c-ba88-4a2ae7217be8,
  6633880a-a907-4713-ab97-8c26d3ee6595, 1339b945-2149-421f-b2f6-14f7babc4693, a4bf69f8-e2c8-447d-8f7e-56772e69ae5f,
  b4fd8305-45ba-4590-a736-8197bbe5cff3, 166f6e14-489d-469b-96fd-b52c67698750, fdf95d41-7573-4c01-8c02-67dec3e70932,
  b993d323-6b0e-4a21-b2bf-3a2a234386cc, 6ad0a7aa-4a9f-4c62-9a7b-d54fd060a938, adaf3199-30fe-42cb-9972-db8e05385499,
  0c1666af-acd9-4641-9280-580be1765365, b76bef74-4c10-4ba7-9261-1d856301656d, de491bf0-6ea8-4347-a109-30b87a6dddad,
  8694d196-d938-4fc8-812d-9c58a58c96bc, f01155be-5153-4a70-939b-98186d7a9e7b, 9e6162da-b6ba-4b51-af75-5e71577dac7d,
  bddf9946-26ac-4dde-b8b5-f5f8458c8a38, 73d10460-c5ad-4bb0-bc14-c3a544b01994, b63fce9a-2053-4f09-9d9d-4aee8cfab15d,
  9ff8cc1f-d497-402f-9164-8abed94506d0, c5815d3c-821f-4c35-9e2a-c824fe9d94d9, 103ec47e-04ac-44fc-bc60-58b7898cfd5f,
  4ea77d01-1585-4982-bf00-971dec77451f, a0df856c-50bc-4a9a-abc5-149bc2f733d3, 95d8888d-0994-46ef-9205-a02c0ad0dd41,
  8520780f-5258-42f1-9cca-5d0448ea58c3, f2276c4e-e22e-4352-a76d-9f244e9f6e9e, 09f8210b-c372-4d4c-83a3-f86b84bcd13b,
  9c1cdbb9-7d7f-43e4-a751-a1410151c840, b8c71526-63ea-49bb-bd1b-d5af1801bcf4, 3d3711cd-ef0f-4543-b852-2cfb3f178bbd,
  158fd367-7d20-42d2-bfdd-6c118d30aebf, 1388dd39-086f-40d6-9a5b-e824397bef70, bfc57036-421e-48c7-80e2-90e967d6f355,
  7da849ab-4965-465d-af76-18e141b81256, 2a4aeb78-0186-4416-9b87-31369d5396f2, abee7eec-8a3c-4f97-b239-6d34c954e488,
  fc8b61aa-50c1-435a-86a2-27b9eb74e165, 83ca153c-dde5-4752-a868-57f5f599d758, f1fad6f3-8b69-4ee5-861b-64a9205a10d6,
  3b8685e6-e917-4c50-9358-a2b196c2ebd3, 0536b4d1-e337-44ac-bfaf-e4100ae90a23, 792a5bef-64ea-46ff-8386-69e93db3aa33,
  e583e7cf-084a-41b2-8810-dd461c695ffe, e1e966ea-050f-4c36-8a1b-c187a50b8d25, ee3fb5bc-a2b5-4c8e-9cbf-243fb8ebc023,
  45a6b2f7-2911-4576-b80d-931a705b2718, 2b03d23d-69b5-4a68-b312-fbd401c21e7b, f35f74fd-d6ae-479d-8833-8a502047ec93,
  6fbd5fa7-c1bd-493d-9a41-02a5e377df79, 5369a3da-b5f5-4726-b095-2e77c372eb4b, 6919e25a-b105-4517-90bf-9e92549cb706,
  4fbf645f-330e-4cc3-8d8e-d5330269b4e5, 12855305-90fd-46be-b5cf-93d669e5c9e8, 8c0d54c1-f40b-4612-82c9-5190bbf54d3a]
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

* Example output for the IML map - Nodes with loaded annotations

```
$rostopic echo /map/nodes
header: 
  seq: 252
  stamp: 
    secs: 127
    nsecs:         0
  frame_id: "map"
info: 
  map_load_time: 
    secs: 0
    nsecs:         0
  resolution: 2.0
  width: 24
  height: 8
  origin: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
x: [3.75, 3.0, 3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 6.300000000000001, 7.0, 7.0, 7.0, 7.0, 9.0, 9.0, 9.0, 9.0, 9.0, 11.0, 11.0, 11.0, 11.0, 11.0, 13.0, 13.0, 13.0, 13.0, 13.0, 15.0, 15.0, 15.0, 15.0, 15.0, 17.96, 17.0, 17.0, 19.0, 19.0, 19.0, 21.0, 21.0, 21.0, 21.0, 21.0, 23.0, 23.0, 23.0, 23.0, 23.0, 25.0, 25.0, 25.0, 27.0, 27.0, 27.0, 29.0, 29.0, 29.0, 29.0, 29.0, 31.0, 31.0, 31.0, 33.0, 35.0, 35.0, 35.0, 37.0, 37.0, 37.0, 37.0, 39.0, 39.0, 39.0, 39.0, 39.0, 41.0, 41.0, 41.0, 43.0, 43.0, 43.0, 45.0, 45.0, 45.0, 47.0, 47.0, 47.0]
y: [2.51, 5.0, 7.0, 9.0, 11.0, 13.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 4.3, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 8.370000000000001, 11.0, 13.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 9.0, 11.0, 13.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 9.0, 9.0, 11.0, 13.0, 5.0, 7.0, 9.0, 11.0, 3.0, 5.0, 7.0, 9.0, 11.0, 3.0, 5.0, 7.0, 3.0, 5.0, 7.0, 3.0, 5.0, 7.0, 3.0, 5.0, 7.0]
theta: [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
name: [loadingArea, vertex_10, vertex_11, vertex_12, vertex_13, vertex_14, vertex_17, vertex_18,
  vertex_19, vertex_20, vertex_21, vertex_22, waitingArea, vertex_27, vertex_28, vertex_29,
  vertex_30, vertex_34, vertex_35, vertex_36, vertex_37, vertex_38, vertex_42, vertex_43,
  vertex_44, vertex_45, vertex_46, vertex_50, vertex_51, vertex_52, vertex_53, vertex_54,
  vertex_58, vertex_59, vertex_60, vertex_61, vertex_62, P1, vertex_69, vertex_70,
  vertex_76, vertex_77, vertex_78, vertex_82, vertex_83, vertex_84, vertex_85, vertex_86,
  vertex_90, vertex_91, vertex_92, vertex_93, vertex_94, vertex_100, vertex_101, vertex_102,
  vertex_108, vertex_109, vertex_110, vertex_114, vertex_115, vertex_116, vertex_117,
  vertex_118, vertex_122, vertex_123, vertex_124, vertex_132, vertex_140, vertex_141,
  vertex_142, vertex_146, vertex_147, vertex_148, vertex_149, vertex_153, vertex_154,
  vertex_155, vertex_156, vertex_157, vertex_161, vertex_162, vertex_163, vertex_169,
  vertex_170, vertex_171, vertex_177, vertex_178, vertex_179, vertex_185, vertex_186,
  vertex_187]
uuid: [42ccd657-e2d1-45db-9194-95bf5943b401, 2da87483-115d-4d27-9cfb-3a2b293d6dc0, 628e1021-859f-49e8-a6d7-b712fa8275ff,
  b141bcec-cae7-4cfa-bd15-d0c2a960f31e, c9043d57-fef8-471b-b858-9741a3ca63e9, f03663ca-73da-4dc3-a413-738f9b088c6c,
  d811065a-12cd-4b75-832a-e196cb9216af, 6b12fd7f-534a-4e81-882b-5db133090f11, 4b5ca285-0bb2-4eb5-931e-ce93a45db973,
  b3dbb45b-e454-40d9-b89a-629abfc71ad0, a1788920-e1c8-4236-98ba-4cac234cb6cd, fa78d668-8dff-4782-b092-8dd236fe4637,
  044b5ead-944d-4ece-aa42-59e156a2da1e, c45b5fba-7aff-46a8-b939-12edc376abda, 256fa77c-acda-451f-aecc-0b7e19a3a67f,
  627cd5ce-f9f7-4093-bd6e-85ee8e6f397c, 649b8875-b732-42d9-87df-cb3cb9d4072b, 5d8fdd4f-ddc0-46bf-97d0-a747a538c63b,
  f340dfc7-201d-47f1-b34e-733c3f224282, 940915cf-ea8e-4e5b-b1ae-733bb9195d80, 48c4d556-48bf-40af-a5e0-d9550bd4efd7,
  55090fff-fb93-4642-84ed-b96a27c336e9, 35904d66-12ef-439e-b7e7-7e74eaa7ebdf, bfef8b5e-bf16-484c-946e-3f61cfbce794,
  65408c0c-fccd-40ac-989f-52ff869f7acb, 2c9bb5e9-6aa2-4877-9ea2-ce3ad95fb820, 508a20be-6bc8-42d3-9acb-91335bc28b84,
  0c64cc03-ef47-412a-ac44-f7f2b5a25a45, 6d042cf5-8db5-444f-a797-c0c564a92a1f, 808a0d5e-c1b0-4d0d-b964-a7908dfe6c8e,
  d06bb68a-3ec1-4096-bc0f-feefd2199327, 98628331-25b2-436b-bf81-2088142e605f, 8aaeeda7-6d08-4f66-b3cf-f63c1b05e7f5,
  cb711d7c-7122-4055-8514-4d28915b2235, aef0ec4a-5be7-4f17-824b-ee9f16795619, 27eb8f85-657c-4f5d-be4b-8c4de00104ba,
  354e0799-5298-4f64-a238-78f2bf682eb3, 8bc21559-6a51-41c8-925c-4fe7b8879618, d7e03e11-ab81-4854-90e6-4d2bca6b1e0f,
  9afb8d88-3d95-4edb-9da9-193e9965b3e1, ed1e896e-87da-46dc-b298-a5713e331aa3, c301eb61-e694-48d6-a9f7-8ed01d090c2a,
  c7bb3122-6600-494f-9a1b-df7fac1764d9, 332b7379-7630-44c8-b0fb-312238176c84, 76c51d96-a630-409f-84d8-23bef60f7972,
  e318d514-a77b-49ed-aa9d-08c8f58ff261, a61601ca-043a-4187-b521-c3561a427963, c0588224-abb7-44ef-940d-d37d8cfbab50,
  be621819-7027-41d0-be27-01bc3d3e082c, 799bc72d-e64f-4d8d-8da1-c284785dc34a, 12e256c2-530a-498d-8676-4efc1ba94cfc,
  32d9d384-678a-424d-9522-002be4002310, e1611ff5-916e-449f-81d8-27e9bb74cf90, 4b8214ee-2231-4941-b1b8-19493951fe8a,
  122d6ce1-1f85-460d-b130-d797b3d8e114, 0defddb5-0695-47d6-b210-db64247a3afc, 9156d5cd-9562-4eff-b2a8-f51016f792ec,
  58b64fab-0d74-4952-8663-5fdafbb00d71, 01d98a4a-639e-4a46-a7b5-b85bf8fea880, 66fa7747-9bdb-4450-915a-bf12ec1c137b,
  cfbdfe3b-46bb-46b4-927c-57a4c40de097, 07bb28e7-96cf-49cd-a204-564b3c20ee62, 74864f5e-2316-4744-87d0-3794324e0864,
  127151ac-f15a-479b-a6f3-3e45b68a3852, 2a2157dc-b4ad-46a1-9e02-9e336c067400, e4d0bb26-9325-49a6-b00c-7a3ce7fc54fa,
  8950ed68-9949-42df-8a72-ac93573a3a1b, 87616f29-a7c5-4812-a635-9f23b3b5f676, 307d2600-0d90-4f9d-85a9-9af9fa23afa1,
  3dca5d7b-adcf-49b5-ba7b-0d97d183397e, f974fa49-009c-4516-8214-c1cd066d7cc2, c5fda5b3-1b2d-49d5-8c1b-3867fd6cf0c2,
  09dff29c-6f5f-4127-8295-8679d5ab6725, ca7d6a16-dac7-42ae-aa7f-76c1658da41f, cdef517d-592a-4cf0-a8b4-60498dfe1dc0,
  a7de0f2a-bdee-4f38-ab30-1b009bda1192, a822e140-46e3-4fed-89af-e0afa5571780, 4bb01e2b-4632-4f12-a78c-8299443b630b,
  f04767eb-7620-46b6-80ea-8874445928d0, 69325631-edeb-4ae5-9438-5c031c10ad53, 90412633-00f4-439e-b7c9-c8a147fbb132,
  5bf9c2bb-0cfd-44fe-ac94-c3e482197d11, 3dbce560-c516-4109-84a3-4fbccc128e3f, e68c2cf6-8235-47e2-9812-2c0e43de6e73,
  52766664-16ba-4d0b-a75f-7df201413c5b, 713d4cdf-b157-4114-8a08-5d1b8f712161, 20e52082-834e-41b9-aff9-7b5ae2c34a4f,
  a1b7c817-d916-4591-bfc7-c3f25b718e44, 2aa2cabf-6deb-4bc2-9c7b-cf97b5553369, 552fe25c-d1e9-4e93-9721-26a9a350ffc0,
  b46ac846-04a6-4a5b-be24-03727ad43469, 4e6a49c5-56b6-44f5-89a5-29f6120ed7ef]
```
* Example output for the IML map - Edges
```
$rostopic echo /map/edges
header: 
  seq: 1503
  stamp: 
    secs: 252
    nsecs: 100000000
  frame_id: "map"
uuid_src: [42ccd657-e2d1-45db-9194-95bf5943b401, 42ccd657-e2d1-45db-9194-95bf5943b401, 2da87483-115d-4d27-9cfb-3a2b293d6dc0,
  2da87483-115d-4d27-9cfb-3a2b293d6dc0, 628e1021-859f-49e8-a6d7-b712fa8275ff, 628e1021-859f-49e8-a6d7-b712fa8275ff,
  b141bcec-cae7-4cfa-bd15-d0c2a960f31e, b141bcec-cae7-4cfa-bd15-d0c2a960f31e, c9043d57-fef8-471b-b858-9741a3ca63e9,
  c9043d57-fef8-471b-b858-9741a3ca63e9, f03663ca-73da-4dc3-a413-738f9b088c6c, d811065a-12cd-4b75-832a-e196cb9216af,
  6b12fd7f-534a-4e81-882b-5db133090f11, 6b12fd7f-534a-4e81-882b-5db133090f11, 4b5ca285-0bb2-4eb5-931e-ce93a45db973,
  4b5ca285-0bb2-4eb5-931e-ce93a45db973, b3dbb45b-e454-40d9-b89a-629abfc71ad0, b3dbb45b-e454-40d9-b89a-629abfc71ad0,
  a1788920-e1c8-4236-98ba-4cac234cb6cd, a1788920-e1c8-4236-98ba-4cac234cb6cd, fa78d668-8dff-4782-b092-8dd236fe4637,
  044b5ead-944d-4ece-aa42-59e156a2da1e, 044b5ead-944d-4ece-aa42-59e156a2da1e, c45b5fba-7aff-46a8-b939-12edc376abda,
  c45b5fba-7aff-46a8-b939-12edc376abda, 256fa77c-acda-451f-aecc-0b7e19a3a67f, 256fa77c-acda-451f-aecc-0b7e19a3a67f,
  627cd5ce-f9f7-4093-bd6e-85ee8e6f397c, 627cd5ce-f9f7-4093-bd6e-85ee8e6f397c, 649b8875-b732-42d9-87df-cb3cb9d4072b,
  5d8fdd4f-ddc0-46bf-97d0-a747a538c63b, 5d8fdd4f-ddc0-46bf-97d0-a747a538c63b, f340dfc7-201d-47f1-b34e-733c3f224282,
  f340dfc7-201d-47f1-b34e-733c3f224282, 940915cf-ea8e-4e5b-b1ae-733bb9195d80, 940915cf-ea8e-4e5b-b1ae-733bb9195d80,
  48c4d556-48bf-40af-a5e0-d9550bd4efd7, 48c4d556-48bf-40af-a5e0-d9550bd4efd7, 55090fff-fb93-4642-84ed-b96a27c336e9,
  35904d66-12ef-439e-b7e7-7e74eaa7ebdf, 35904d66-12ef-439e-b7e7-7e74eaa7ebdf, bfef8b5e-bf16-484c-946e-3f61cfbce794,
  bfef8b5e-bf16-484c-946e-3f61cfbce794, 65408c0c-fccd-40ac-989f-52ff869f7acb, 65408c0c-fccd-40ac-989f-52ff869f7acb,
  2c9bb5e9-6aa2-4877-9ea2-ce3ad95fb820, 2c9bb5e9-6aa2-4877-9ea2-ce3ad95fb820, 508a20be-6bc8-42d3-9acb-91335bc28b84,
  0c64cc03-ef47-412a-ac44-f7f2b5a25a45, 0c64cc03-ef47-412a-ac44-f7f2b5a25a45, 6d042cf5-8db5-444f-a797-c0c564a92a1f,
  6d042cf5-8db5-444f-a797-c0c564a92a1f, 808a0d5e-c1b0-4d0d-b964-a7908dfe6c8e, 808a0d5e-c1b0-4d0d-b964-a7908dfe6c8e,
  d06bb68a-3ec1-4096-bc0f-feefd2199327, d06bb68a-3ec1-4096-bc0f-feefd2199327, 98628331-25b2-436b-bf81-2088142e605f,
  8aaeeda7-6d08-4f66-b3cf-f63c1b05e7f5, cb711d7c-7122-4055-8514-4d28915b2235, aef0ec4a-5be7-4f17-824b-ee9f16795619,
  aef0ec4a-5be7-4f17-824b-ee9f16795619, 27eb8f85-657c-4f5d-be4b-8c4de00104ba, 27eb8f85-657c-4f5d-be4b-8c4de00104ba,
  354e0799-5298-4f64-a238-78f2bf682eb3, 8bc21559-6a51-41c8-925c-4fe7b8879618, 8bc21559-6a51-41c8-925c-4fe7b8879618,
  d7e03e11-ab81-4854-90e6-4d2bca6b1e0f, d7e03e11-ab81-4854-90e6-4d2bca6b1e0f, 9afb8d88-3d95-4edb-9da9-193e9965b3e1,
  ed1e896e-87da-46dc-b298-a5713e331aa3, ed1e896e-87da-46dc-b298-a5713e331aa3, c301eb61-e694-48d6-a9f7-8ed01d090c2a,
  c301eb61-e694-48d6-a9f7-8ed01d090c2a, c7bb3122-6600-494f-9a1b-df7fac1764d9, 332b7379-7630-44c8-b0fb-312238176c84,
  332b7379-7630-44c8-b0fb-312238176c84, 76c51d96-a630-409f-84d8-23bef60f7972, 76c51d96-a630-409f-84d8-23bef60f7972,
  e318d514-a77b-49ed-aa9d-08c8f58ff261, e318d514-a77b-49ed-aa9d-08c8f58ff261, a61601ca-043a-4187-b521-c3561a427963,
  a61601ca-043a-4187-b521-c3561a427963, c0588224-abb7-44ef-940d-d37d8cfbab50, be621819-7027-41d0-be27-01bc3d3e082c,
  799bc72d-e64f-4d8d-8da1-c284785dc34a, 12e256c2-530a-498d-8676-4efc1ba94cfc, 12e256c2-530a-498d-8676-4efc1ba94cfc,
  32d9d384-678a-424d-9522-002be4002310, 32d9d384-678a-424d-9522-002be4002310, e1611ff5-916e-449f-81d8-27e9bb74cf90,
  4b8214ee-2231-4941-b1b8-19493951fe8a, 4b8214ee-2231-4941-b1b8-19493951fe8a, 122d6ce1-1f85-460d-b130-d797b3d8e114,
  122d6ce1-1f85-460d-b130-d797b3d8e114, 0defddb5-0695-47d6-b210-db64247a3afc, 9156d5cd-9562-4eff-b2a8-f51016f792ec,
  9156d5cd-9562-4eff-b2a8-f51016f792ec, 58b64fab-0d74-4952-8663-5fdafbb00d71, 58b64fab-0d74-4952-8663-5fdafbb00d71,
  01d98a4a-639e-4a46-a7b5-b85bf8fea880, 66fa7747-9bdb-4450-915a-bf12ec1c137b, 66fa7747-9bdb-4450-915a-bf12ec1c137b,
  cfbdfe3b-46bb-46b4-927c-57a4c40de097, cfbdfe3b-46bb-46b4-927c-57a4c40de097, 07bb28e7-96cf-49cd-a204-564b3c20ee62,
  07bb28e7-96cf-49cd-a204-564b3c20ee62, 74864f5e-2316-4744-87d0-3794324e0864, 2a2157dc-b4ad-46a1-9e02-9e336c067400,
  e4d0bb26-9325-49a6-b00c-7a3ce7fc54fa, 8950ed68-9949-42df-8a72-ac93573a3a1b, 87616f29-a7c5-4812-a635-9f23b3b5f676,
  307d2600-0d90-4f9d-85a9-9af9fa23afa1, 307d2600-0d90-4f9d-85a9-9af9fa23afa1, 3dca5d7b-adcf-49b5-ba7b-0d97d183397e,
  3dca5d7b-adcf-49b5-ba7b-0d97d183397e, c5fda5b3-1b2d-49d5-8c1b-3867fd6cf0c2, c5fda5b3-1b2d-49d5-8c1b-3867fd6cf0c2,
  09dff29c-6f5f-4127-8295-8679d5ab6725, 09dff29c-6f5f-4127-8295-8679d5ab6725, ca7d6a16-dac7-42ae-aa7f-76c1658da41f,
  ca7d6a16-dac7-42ae-aa7f-76c1658da41f, cdef517d-592a-4cf0-a8b4-60498dfe1dc0, a7de0f2a-bdee-4f38-ab30-1b009bda1192,
  a7de0f2a-bdee-4f38-ab30-1b009bda1192, a822e140-46e3-4fed-89af-e0afa5571780, a822e140-46e3-4fed-89af-e0afa5571780,
  4bb01e2b-4632-4f12-a78c-8299443b630b, 4bb01e2b-4632-4f12-a78c-8299443b630b, f04767eb-7620-46b6-80ea-8874445928d0,
  90412633-00f4-439e-b7c9-c8a147fbb132, 90412633-00f4-439e-b7c9-c8a147fbb132, 5bf9c2bb-0cfd-44fe-ac94-c3e482197d11,
  5bf9c2bb-0cfd-44fe-ac94-c3e482197d11, 3dbce560-c516-4109-84a3-4fbccc128e3f, e68c2cf6-8235-47e2-9812-2c0e43de6e73,
  e68c2cf6-8235-47e2-9812-2c0e43de6e73, 52766664-16ba-4d0b-a75f-7df201413c5b, 52766664-16ba-4d0b-a75f-7df201413c5b,
  713d4cdf-b157-4114-8a08-5d1b8f712161, 20e52082-834e-41b9-aff9-7b5ae2c34a4f, 20e52082-834e-41b9-aff9-7b5ae2c34a4f,
  a1b7c817-d916-4591-bfc7-c3f25b718e44, a1b7c817-d916-4591-bfc7-c3f25b718e44, 2aa2cabf-6deb-4bc2-9c7b-cf97b5553369,
  552fe25c-d1e9-4e93-9721-26a9a350ffc0, b46ac846-04a6-4a5b-be24-03727ad43469]
uuid_dest: [d811065a-12cd-4b75-832a-e196cb9216af, 2da87483-115d-4d27-9cfb-3a2b293d6dc0, 6b12fd7f-534a-4e81-882b-5db133090f11,
  628e1021-859f-49e8-a6d7-b712fa8275ff, 4b5ca285-0bb2-4eb5-931e-ce93a45db973, b141bcec-cae7-4cfa-bd15-d0c2a960f31e,
  b3dbb45b-e454-40d9-b89a-629abfc71ad0, c9043d57-fef8-471b-b858-9741a3ca63e9, a1788920-e1c8-4236-98ba-4cac234cb6cd,
  f03663ca-73da-4dc3-a413-738f9b088c6c, fa78d668-8dff-4782-b092-8dd236fe4637, 6b12fd7f-534a-4e81-882b-5db133090f11,
  044b5ead-944d-4ece-aa42-59e156a2da1e, 4b5ca285-0bb2-4eb5-931e-ce93a45db973, c45b5fba-7aff-46a8-b939-12edc376abda,
  b3dbb45b-e454-40d9-b89a-629abfc71ad0, 256fa77c-acda-451f-aecc-0b7e19a3a67f, a1788920-e1c8-4236-98ba-4cac234cb6cd,
  627cd5ce-f9f7-4093-bd6e-85ee8e6f397c, fa78d668-8dff-4782-b092-8dd236fe4637, 649b8875-b732-42d9-87df-cb3cb9d4072b,
  5d8fdd4f-ddc0-46bf-97d0-a747a538c63b, c45b5fba-7aff-46a8-b939-12edc376abda, f340dfc7-201d-47f1-b34e-733c3f224282,
  256fa77c-acda-451f-aecc-0b7e19a3a67f, 940915cf-ea8e-4e5b-b1ae-733bb9195d80, 627cd5ce-f9f7-4093-bd6e-85ee8e6f397c,
  48c4d556-48bf-40af-a5e0-d9550bd4efd7, 649b8875-b732-42d9-87df-cb3cb9d4072b, 55090fff-fb93-4642-84ed-b96a27c336e9,
  35904d66-12ef-439e-b7e7-7e74eaa7ebdf, f340dfc7-201d-47f1-b34e-733c3f224282, bfef8b5e-bf16-484c-946e-3f61cfbce794,
  940915cf-ea8e-4e5b-b1ae-733bb9195d80, 65408c0c-fccd-40ac-989f-52ff869f7acb, 48c4d556-48bf-40af-a5e0-d9550bd4efd7,
  2c9bb5e9-6aa2-4877-9ea2-ce3ad95fb820, 55090fff-fb93-4642-84ed-b96a27c336e9, 508a20be-6bc8-42d3-9acb-91335bc28b84,
  0c64cc03-ef47-412a-ac44-f7f2b5a25a45, bfef8b5e-bf16-484c-946e-3f61cfbce794, 6d042cf5-8db5-444f-a797-c0c564a92a1f,
  65408c0c-fccd-40ac-989f-52ff869f7acb, 808a0d5e-c1b0-4d0d-b964-a7908dfe6c8e, 2c9bb5e9-6aa2-4877-9ea2-ce3ad95fb820,
  d06bb68a-3ec1-4096-bc0f-feefd2199327, 508a20be-6bc8-42d3-9acb-91335bc28b84, 98628331-25b2-436b-bf81-2088142e605f,
  8aaeeda7-6d08-4f66-b3cf-f63c1b05e7f5, 6d042cf5-8db5-444f-a797-c0c564a92a1f, cb711d7c-7122-4055-8514-4d28915b2235,
  808a0d5e-c1b0-4d0d-b964-a7908dfe6c8e, aef0ec4a-5be7-4f17-824b-ee9f16795619, d06bb68a-3ec1-4096-bc0f-feefd2199327,
  27eb8f85-657c-4f5d-be4b-8c4de00104ba, 98628331-25b2-436b-bf81-2088142e605f, 354e0799-5298-4f64-a238-78f2bf682eb3,
  cb711d7c-7122-4055-8514-4d28915b2235, aef0ec4a-5be7-4f17-824b-ee9f16795619, 8bc21559-6a51-41c8-925c-4fe7b8879618,
  27eb8f85-657c-4f5d-be4b-8c4de00104ba, d7e03e11-ab81-4854-90e6-4d2bca6b1e0f, 354e0799-5298-4f64-a238-78f2bf682eb3,
  9afb8d88-3d95-4edb-9da9-193e9965b3e1, ed1e896e-87da-46dc-b298-a5713e331aa3, d7e03e11-ab81-4854-90e6-4d2bca6b1e0f,
  c301eb61-e694-48d6-a9f7-8ed01d090c2a, 9afb8d88-3d95-4edb-9da9-193e9965b3e1, c7bb3122-6600-494f-9a1b-df7fac1764d9,
  e318d514-a77b-49ed-aa9d-08c8f58ff261, c301eb61-e694-48d6-a9f7-8ed01d090c2a, a61601ca-043a-4187-b521-c3561a427963,
  c7bb3122-6600-494f-9a1b-df7fac1764d9, c0588224-abb7-44ef-940d-d37d8cfbab50, be621819-7027-41d0-be27-01bc3d3e082c,
  76c51d96-a630-409f-84d8-23bef60f7972, 799bc72d-e64f-4d8d-8da1-c284785dc34a, e318d514-a77b-49ed-aa9d-08c8f58ff261,
  12e256c2-530a-498d-8676-4efc1ba94cfc, a61601ca-043a-4187-b521-c3561a427963, 32d9d384-678a-424d-9522-002be4002310,
  c0588224-abb7-44ef-940d-d37d8cfbab50, e1611ff5-916e-449f-81d8-27e9bb74cf90, 799bc72d-e64f-4d8d-8da1-c284785dc34a,
  12e256c2-530a-498d-8676-4efc1ba94cfc, 4b8214ee-2231-4941-b1b8-19493951fe8a, 32d9d384-678a-424d-9522-002be4002310,
  122d6ce1-1f85-460d-b130-d797b3d8e114, e1611ff5-916e-449f-81d8-27e9bb74cf90, 0defddb5-0695-47d6-b210-db64247a3afc,
  9156d5cd-9562-4eff-b2a8-f51016f792ec, 122d6ce1-1f85-460d-b130-d797b3d8e114, 58b64fab-0d74-4952-8663-5fdafbb00d71,
  0defddb5-0695-47d6-b210-db64247a3afc, 01d98a4a-639e-4a46-a7b5-b85bf8fea880, 07bb28e7-96cf-49cd-a204-564b3c20ee62,
  58b64fab-0d74-4952-8663-5fdafbb00d71, 74864f5e-2316-4744-87d0-3794324e0864, 01d98a4a-639e-4a46-a7b5-b85bf8fea880,
  127151ac-f15a-479b-a6f3-3e45b68a3852, 2a2157dc-b4ad-46a1-9e02-9e336c067400, cfbdfe3b-46bb-46b4-927c-57a4c40de097,
  e4d0bb26-9325-49a6-b00c-7a3ce7fc54fa, 07bb28e7-96cf-49cd-a204-564b3c20ee62, 8950ed68-9949-42df-8a72-ac93573a3a1b,
  74864f5e-2316-4744-87d0-3794324e0864, 127151ac-f15a-479b-a6f3-3e45b68a3852, e4d0bb26-9325-49a6-b00c-7a3ce7fc54fa,
  8950ed68-9949-42df-8a72-ac93573a3a1b, 87616f29-a7c5-4812-a635-9f23b3b5f676, 307d2600-0d90-4f9d-85a9-9af9fa23afa1,
  ca7d6a16-dac7-42ae-aa7f-76c1658da41f, 3dca5d7b-adcf-49b5-ba7b-0d97d183397e, cdef517d-592a-4cf0-a8b4-60498dfe1dc0,
  f974fa49-009c-4516-8214-c1cd066d7cc2, a822e140-46e3-4fed-89af-e0afa5571780, 09dff29c-6f5f-4127-8295-8679d5ab6725,
  4bb01e2b-4632-4f12-a78c-8299443b630b, ca7d6a16-dac7-42ae-aa7f-76c1658da41f, f04767eb-7620-46b6-80ea-8874445928d0,
  cdef517d-592a-4cf0-a8b4-60498dfe1dc0, 69325631-edeb-4ae5-9438-5c031c10ad53, 90412633-00f4-439e-b7c9-c8a147fbb132,
  a822e140-46e3-4fed-89af-e0afa5571780, 5bf9c2bb-0cfd-44fe-ac94-c3e482197d11, 4bb01e2b-4632-4f12-a78c-8299443b630b,
  3dbce560-c516-4109-84a3-4fbccc128e3f, f04767eb-7620-46b6-80ea-8874445928d0, 69325631-edeb-4ae5-9438-5c031c10ad53,
  e68c2cf6-8235-47e2-9812-2c0e43de6e73, 5bf9c2bb-0cfd-44fe-ac94-c3e482197d11, 52766664-16ba-4d0b-a75f-7df201413c5b,
  3dbce560-c516-4109-84a3-4fbccc128e3f, 713d4cdf-b157-4114-8a08-5d1b8f712161, 20e52082-834e-41b9-aff9-7b5ae2c34a4f,
  52766664-16ba-4d0b-a75f-7df201413c5b, a1b7c817-d916-4591-bfc7-c3f25b718e44, 713d4cdf-b157-4114-8a08-5d1b8f712161,
  2aa2cabf-6deb-4bc2-9c7b-cf97b5553369, 552fe25c-d1e9-4e93-9721-26a9a350ffc0, a1b7c817-d916-4591-bfc7-c3f25b718e44,
  b46ac846-04a6-4a5b-be24-03727ad43469, 2aa2cabf-6deb-4bc2-9c7b-cf97b5553369, 4e6a49c5-56b6-44f5-89a5-29f6120ed7ef,
  b46ac846-04a6-4a5b-be24-03727ad43469, 4e6a49c5-56b6-44f5-89a5-29f6120ed7ef]
name: [edge_9_17, edge_9_10, edge_10_18, edge_10_11, edge_11_19, edge_11_12, edge_12_20,
  edge_12_13, edge_13_21, edge_13_14, edge_14_22, edge_17_18, edge_18_26, edge_18_19,
  edge_19_27, edge_19_20, edge_20_28, edge_20_21, edge_21_29, edge_21_22, edge_22_30,
  edge_26_34, edge_26_27, edge_27_35, edge_27_28, edge_28_36, edge_28_29, edge_29_37,
  edge_29_30, edge_30_38, edge_34_42, edge_34_35, edge_35_43, edge_35_36, edge_36_44,
  edge_36_37, edge_37_45, edge_37_38, edge_38_46, edge_42_50, edge_42_43, edge_43_51,
  edge_43_44, edge_44_52, edge_44_45, edge_45_53, edge_45_46, edge_46_54, edge_50_58,
  edge_50_51, edge_51_59, edge_51_52, edge_52_60, edge_52_53, edge_53_61, edge_53_54,
  edge_54_62, edge_58_59, edge_59_60, edge_60_68, edge_60_61, edge_61_69, edge_61_62,
  edge_62_70, edge_68_76, edge_68_69, edge_69_77, edge_69_70, edge_70_78, edge_76_84,
  edge_76_77, edge_77_85, edge_77_78, edge_78_86, edge_82_90, edge_82_83, edge_83_91,
  edge_83_84, edge_84_92, edge_84_85, edge_85_93, edge_85_86, edge_86_94, edge_90_91,
  edge_91_92, edge_92_100, edge_92_93, edge_93_101, edge_93_94, edge_94_102, edge_100_108,
  edge_100_101, edge_101_109, edge_101_102, edge_102_110, edge_108_116, edge_108_109,
  edge_109_117, edge_109_110, edge_110_118, edge_114_122, edge_114_115, edge_115_123,
  edge_115_116, edge_116_124, edge_116_117, edge_117_118, edge_122_123, edge_123_124,
  edge_124_132, edge_132_140, edge_140_148, edge_140_141, edge_141_149, edge_141_142,
  edge_146_154, edge_146_147, edge_147_155, edge_147_148, edge_148_156, edge_148_149,
  edge_149_157, edge_153_161, edge_153_154, edge_154_162, edge_154_155, edge_155_163,
  edge_155_156, edge_156_157, edge_161_169, edge_161_162, edge_162_170, edge_162_163,
  edge_163_171, edge_169_177, edge_169_170, edge_170_178, edge_170_171, edge_171_179,
  edge_177_185, edge_177_178, edge_178_186, edge_178_179, edge_179_187, edge_185_186,
  edge_186_187]
uuid: [3d47a8cf-cbdc-407f-99d1-9738844b7a79, edb55269-fb68-4f3a-99c5-d41b5fdbbc2e, 4f73e648-ef47-4990-8e8f-d7f56597b278,
  81c0fa45-ae0b-44ad-9bc6-eb87941db657, 148a2aea-113f-4cf3-bae4-056fe019ac35, a1a501d9-ed4d-4a97-96cf-4d7b5d89b796,
  d534777f-8fff-4fc4-a1ba-fffdcf9118b8, 39045629-b7a7-42bf-b82f-4f587b110623, a812eb46-817f-4487-b9dc-2444920e2c92,
  b06dcdc0-7b16-4276-9166-a051ccd8a66f, df638620-ec7e-4924-b67b-aa4603a3e319, 4e67d023-efc3-4b1c-90fd-a7ee583b7c3d,
  060170dc-e982-4c34-a9c5-5027eb19cca3, e6dfc93c-825d-42e0-8300-b5a2780f5d14, 7ce68e99-b116-4560-8924-effa6842ad31,
  179e9508-db12-4572-aa8f-c2708094eb23, 849d6e21-222a-4473-8cb0-6f0187a7da0b, ad5b206e-4789-496e-97bc-0ec7c0426076,
  e1f18224-a5df-495a-b14f-05252be8fca4, eef12358-436b-4d66-a9a6-641be0d0d930, 0268f699-1648-438f-99e7-8a4a8c3313eb,
  c387204d-337b-4a04-85ec-84c6a196945d, a4215039-3f6b-4e49-a3af-94bd8fbaf128, a3ed6cbb-b932-4ee8-91cc-4e0671d4953e,
  82506f48-17d6-4765-8722-e9ee3adf3563, 80b2526a-ebb8-4dce-9c0c-33454315a591, 2a84aff8-2bc9-411f-bdc5-b334f65879f5,
  0713b07b-2e01-4cdc-80b1-6d36625d658d, d1e309f6-8ecb-4c76-98cd-c29b4c0d882e, 0c2a4753-8be4-4208-b62f-9c6dba5b9c31,
  9014c030-9666-4da7-97da-da224a78a21a, ce4ae4b3-7996-4a56-b2ca-7d6bc81fca12, 4efc76a9-feba-4ec7-b946-8caaff20424d,
  51f343ff-b811-482b-ac95-4af8c0cfe32d, c7f4e307-c5d4-4c48-8cf1-9f56abbf04f6, c2522679-2e1c-4cfd-9ea8-0b7d5cc1ae68,
  d0a11ba1-648f-4e03-b2ee-8e4605b19595, 90b057f4-ad08-48c0-baf1-fab90108d062, 06eb80c1-2547-4376-a07c-d9cb8b00b4f3,
  d7f5cd46-e376-48e4-9f0a-37d493560d58, 1e306df1-18b6-427a-92cc-b0dc66b84516, 59c29936-4af1-4d83-a53b-d88d319ad78e,
  3f93d178-0497-4a9b-9147-4a3cc3163513, 6ac2f4f7-7c4a-4b1e-a2ca-b9fb39ae1757, 6c80903f-0bae-4157-a204-fd553a6e95ba,
  ac2d808d-2e77-4818-92e1-fd52f81f20ee, e036a607-1a6a-490a-bdee-4dbe8662b50d, 28a4023a-09a1-4153-b3eb-0dd7276935fb,
  57ebd20d-ee27-4b57-89cb-a688b42023b9, 03fc26dd-41f2-4134-bde3-bc9cf3bb9c18, 1d81737e-f3bd-471f-97ff-6105b5287c77,
  27d0ecfc-07a2-4ddb-ad4b-237cd04cf682, cdbd5624-a15f-4a3c-a614-770d958de46e, f3541a87-6740-4303-b54f-cd5a7c1cad48,
  4fc29335-1d2b-44e8-9277-452a7267c2f8, b899b091-5f17-41ed-80e7-15f828826aa8, 22053e5a-4a49-49da-98aa-3adedaa83a86,
  69321811-d4f5-482e-b42e-4554251d3ed5, b704965d-d7c3-4851-aa75-7bb668623985, 5e924ff3-c4d8-4863-b4e7-4b3fdc40d3a2,
  1ab6ddff-fa37-4f80-b610-fda495f7bbb2, 70001515-17ee-4cd1-9b06-6ff267dd0425, 41382357-c6c1-4742-a411-dd95ce750f36,
  baad77b4-4235-4e5d-a621-84dd48390f69, cbbddc95-9dfc-44f7-9d66-909f654a4dce, f5231388-0df3-4982-9f8e-0e2ac57ec82e,
  e7188b6f-8a7c-41fa-a8a1-632012da44d6, df5f9e84-b78b-457f-bd3a-f7b208f75196, 42c2c3bf-3ef8-40d0-9bdb-a2e3c9398e5d,
  96ac9e9f-cf24-4aa0-b9ab-65e2458620ae, 5c9002a2-c960-4734-8a66-346a61b8a314, a602bf46-936b-4a2e-951d-fe6597505e1d,
  a5610163-6148-4331-9a1c-c3fe18660bf5, 57693afd-0c82-4b25-bf75-4aa8759c2090, 3872f88c-2445-45f3-ad3b-72fdf213fbdd,
  d4e31c87-e5f7-49de-95a2-2f6d5cc7b88a, 28ad873c-b69f-4576-9f12-4c081a435d4b, fca4ecf5-4b3f-4e62-ac09-4ac347294513,
  2d52768d-b66c-44ce-b701-9d27c6d79ce3, 6324208f-9a84-49aa-aca7-063ba2a6f1d7, 5e23f7e3-972b-4f96-95e7-0987661a1672,
  d9df360f-3859-457c-a689-25a55f3a21fb, 58dd97e5-2b91-4b01-bda7-080206c6f319, df8ad300-c7b9-4252-928b-e53bed8916f2,
  638ff052-e8bb-4260-8180-71512a375b38, 207bade6-d851-4182-9738-0c11eeb3b1af, b2c82ce6-fefe-4a11-8953-b2ecb049f0e1,
  5ad0fff5-7161-49fb-8ad2-63107f31632f, d6b295e9-19eb-4361-a6db-663f3db117a7, 09089963-a8ba-4ecb-88d6-a2a36653d4f2,
  97897ba9-4cdb-459e-b33b-a1149f82d461, 67de5415-bd13-49f5-82ce-0b8784839811, 154d71fb-bf8c-4566-baa7-befd6394eba4,
  9148eee4-535c-4cb1-810e-93d4e837a12a, 76411973-19e2-4d8a-b00a-e9309c2ebce6, f327d597-6ccb-49ff-b596-7dc993d6429d,
  16a34be5-e8d9-4a21-b23a-95be674f9b55, 1207996c-22d7-40de-b8fc-19022b33b2ab, 5e794536-922f-42b0-848f-79ea5f4eb4aa,
  5d624c5a-e5ab-4648-a59e-e9ef6542b76d, 83ed3c14-1641-4ca8-ada1-c8f6040ad612, 6ed02c91-7348-485c-8248-5f759c3a77b3,
  9b3d5965-11af-4cd3-92d5-ccf6734b33f8, 89b72334-0374-49ea-acf0-217d11f89701, 770897fe-c124-4733-9d7b-dbe15b30342a,
  033a5ce3-9591-4138-81e6-484ced0bb59d, efa20e31-212d-481a-bfd4-9c0e32b3da21, fe5e0953-4a0c-4bf9-9d8a-64b473caf9cf,
  da549964-81cb-4afb-922c-dddc8bff6b99, 260083f0-5f26-4336-9f6d-b9fe1deef9ea, afc828e5-43be-460d-ace3-48988c69d28a,
  f4a4ad7d-4da1-4b52-be91-e8e2baa31bd9, a3a671d5-8a75-468e-9ef1-abf62d1b1587, f7738b6e-abff-4676-8e56-7e617e2bd26d,
  4b19ff57-9bbd-4196-b83a-d35721649e5d, 801d2def-f3e8-478d-874a-bd84a203841f, d602879b-3543-408c-ac82-aaa9a74463b9,
  792423e8-6b59-4611-8108-d4272ba7597d, 71f872dc-3268-4bf1-b3ff-30b6026737db, cd7d189e-69a8-4036-ad4c-9e618ca7d992,
  814fd7ee-1a20-4055-92a3-dfc91c8ef812, b2531aa8-ef59-4c12-b2f8-fe78c3dd471d, 450509ac-7772-4d4c-afa0-bc5769c2a8d9,
  b0f61ac7-2304-4273-98b2-61b28b2ef418, aa58ca41-cb13-4d1f-9b80-7b5929a9de59, 9d43642d-8a3a-48f7-be94-8665634f38c9,
  8f43219d-e609-4366-8f44-b1c9e6a2c671, e3d67fe4-9980-44bc-b1d0-bf28c8e0d242, 2615a549-35b1-400d-8410-c8b49e1098c5,
  6a777420-63f4-45be-971b-41d358247b28, 48a43ec1-6237-4b18-a5cc-26b403e1996a, 8f7c102e-bda0-447c-97a6-e25a4f8627d2,
  15791cdf-e191-422c-a18e-3de179613fdf, c14c3ca2-88be-411e-980d-6666891c8a5c, 2f28a94a-b638-47e6-b3a8-2d28e5af3598,
  4c28fb2c-bb7e-4305-b1ed-e8c809cd5f9a, c3033dba-89b3-4feb-919c-ed1deab8377a, c3be0af1-f503-49b6-bfb7-bbef902a2bfd,
  c09f1d2e-557a-4fcd-84ef-c43581b5aa68, 117c76d4-64b0-4515-a2c7-435c99f81ed0, 416d622a-2853-4f70-aea0-3391df7677ca,
  771c2a7c-b11f-42c3-98a0-d5a1af30357f, baf15bc6-9c7b-4938-a4a2-6a5b6e2dc8ef, 12aa7ae3-3bf0-46b8-ab4e-4847afb10845,
  834f7a7e-0a6b-4ef2-8790-1286679d5c5e, 5f93d50a-561d-48f0-92b5-f0b34b771383]
```
* Example output for the IML map - Pose with covariance
```
$rostopic echo /robot_0/pose_channel 
header: 
  seq: 76
  stamp: 
    secs: 36
    nsecs: 200000000
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 15.9636536984
      y: 13.3448596722
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.718399874794
      w: 0.695630375915
  covariance: [0.23681326809196435, -0.0016609806341989497, 0.0, 0.0, 0.0, 0.0, -0.0016609806341989497, 0.23012575436405314, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06503792236623226]
```



