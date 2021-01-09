In the following the list of ROS packages belonging to the Central SP is given with the short explanation.
Afterwards, each package is explained with the followed examples.

# Overview of ROS packages

All ROS packages are in src folder. Here are explained only the ones belonging to the Central SP.


## maptogridmap package

A ROS package for topology creation based on a gridmap with desired cell size from _map_server_ (PNG or PGM file) and merging the local map updates from up to three AGVs into a topology. Additionally, annotations are loaded and written into the topology for the Task Planner.

## maplistener package

A ROS package for testing subscribers to all created topics from maptogridmap package and visualizing them in rviz.





# <a name="topology">Topology creation - maptogridmap package</a>
This package creates the output topic **graph** composed of arrays of **edges** and **vertices**. Vertices have coordinates (**x**, **y**) of the cell center defined by the parameter **cell_size** (2 m in this example), a string **name** that describes the vertex, and a string **uuid** uniquely identifying the vertex. The example of one vertex has the following structure:
```
    x: 21.0
    y: 7.0
    theta: 0.0
    name: "vertex_73"
    uuid: "d3ef3744-18cc-551b-80b9-95ef2095919e"
```
**Edges** are pairs of neighbor vertices and are composed of **uuid_src** of the first vertex in the pair, and **uuid_dest** of the second vertex in the pair. The order of the first and second vertex is irrelevant since edges are bidirectional. There is a string **name** of the edge describing the connection of vertices, and a unique **uuid** of the edge. The example of one edge has the following structure:
```
    uuid_src: "27819669-6b67-57e9-a412-d0226f0d4f7e"
    uuid_dest: "cb94a6e7-2531-5285-a090-cdac96307386"
    name: "edge_8_15"
    uuid: "a41a0772-9407-5d31-b1dd-c03c3d27a7f7"
```

It is created as a ROS message composed of arrays of two ROS messages: Vertex and Edge.

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

Annotations can be defined through RViz **2D Nav Goal** button. This button listens to the topic `/annotation/goal`.
Anternatively, annotations can be loaded from file annotations.ini located in maptogridmap/launch folder, which is put as a parameter textfile inside the startmaptogridmap.launch file. In this example the first three annotations were used in Zagreb demo in Task planner, and P1 is put as an additional example for the IML map:
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
The annotations are saved under the variable of type maptogridmap::Annotations inside of the maptogridmap package:

Annotations.msg

	Header header	# standard ROS header
	Annotation[] annotations

Annotation.msg

	float64 x
	float64 y
	float64 theta
	float64 distance
	string name
	string uuid	

All values must be in meters and degrees. From these values it is calculated where the AGV needs to be placed in front of the annotation according to the distance from the annotation and the orientation theta. It changes the values of the computed nodes from gridmap cells so that TP can use this nodes as goals.

These four annotations change the coordinates of the cell centre of the grid map (but only free cells) and also change the name to the annotation name, e.g. loadingArea, unloadingArea, etc. The result can be seen in [topic /map/graph](#exampleannot), while annotations can be seen in topic /map/annotations.

The additional example explains the creation of one annotation in the topology graph. First, create the following file and save it under the name `annotations.ini`:
```
#annotations.ini
[P1]
# coordinates
point_x = 18.4
point_y = 6.5
theta = 180
distance = 1.8
```
where **P1** is the annotation label, **point_x**, **point_y** are coordinates of the annotation, and **theta** and **distance** determine where the topology vertex should be so that Task Planner can use this coordinates as the goal distanced for a defined **distance** (1.8 m in this example) from the annotation and oriented towards the annotation so that AGV has heading **theta** (180 degrees in this example) with respect to the positive x-axis. To use the created `annotations.ini`, save it as maptogridmap/launch/annotations.ini and run again:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
The example of the changed vertex in the graph has now the following structure:
```
    x: 20.2
    y: 6.5
    theta: 180.0
    name: "P1"
    uuid: "d3ef3744-18cc-551b-80b9-95ef2095919e"
``` 
Notice that coordinates of the cell center at (21,7) moved to (20.2,6.5) with respect to given **distance** and **theta** from the annotation, and the **name** has changed from `vertex_73` to `P1`. 


## Creation of Edges in maptogridmap package
Edges are pairs of neighbor nodes. Neighbors are defined between two nodes which have their centres' coordinates distanced for _cell_size_. The edges are bidirectional, meaning two neighbor nodes n and m forms the edge (n,m) and (m,n) which are identical.
Values of Edges are the source node's uuid, named as _uuid_src_, the destination node's uuid, named as _uuid_dest_, the name of the edge in the form of "edge_0_1" meaning that two nodes with names "vertex_0" and "vertex_1" are connected with the edge, and the edge's uuid. 
The message that is sent through firos is maptogridmap/msg/Graph.msg

## <a name="writelis">Writing a simple listener explaining the maplistener package</a>

To read the topic in your own package you need to subscribe to it, include the header of the message, and write a message callback. The example is taken from maplistener/src/main.cpp.

* subscribe to a topic /map/graph:
```
 	graph_sub = nh_.subscribe("map/graph",1,&VisualizationPublisherGML::graphCallback, this);

```
* include the header of the message in your header file or in the cpp where you are writing the message callback:
```
#include <maptogridmap/Graph.h>
```
* write a message callback for Graph message, which draws a small square at each vertex and draw lines for each edge:
```
void VisualizationPublisherGML::graphCallback(const maptogridmap::GraphConstPtr& gmMsg)
{
  graphvertex.points.clear();
  stc.points.clear();
  geometry_msgs::Point p; 
	for (uint i=0; i<gmMsg->vertices.size(); i++){
		p.x=gmMsg->vertices[i].x;
		p.y=gmMsg->vertices[i].y;
		graphvertex.points.push_back(p);
	}
	for (uint i=0; i<gmMsg->edges.size(); i++){
		int foundsrcdest=0;
		for (uint j=0; j<gmMsg->vertices.size(); j++){
			if ((gmMsg->vertices[j].uuid==gmMsg->edges[i].uuid_src)||(gmMsg->vertices[j].uuid==gmMsg->edges[i].uuid_dest)){
				p.x=gmMsg->vertices[j].x;
				p.y=gmMsg->vertices[j].y;
				stc.points.push_back(p);
				foundsrcdest++;
				if (foundsrcdest==2)
					break;
			}
		}
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



## <a name="exampleannot">Example output of the topology graph topic with the loaded annotations</a>
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
  - 
    x: 4.0
    y: 4.0
    theta: 0.0
    name: "vertex_10"
    uuid: "3dcd4aec-1a4f-5156-a517-17cdcf55ad47"
  - 
    x: 5.6
    y: 2.4
    theta: 0.0
    name: "vertex_13"
    uuid: "45a28b12-3dc3-5dc3-a687-14fc7fa3be89"
  - 
    x: 5.6
    y: 4.0
    theta: 0.0
    name: "vertex_14"
    uuid: "439be9e5-a52a-5cea-88e2-ab6150ffe578"
  - 
    x: 7.0
    y: 2.51
    theta: 0.0
    name: "unloadingArea"
    uuid: "9ba5016b-e219-513f-af73-ebce59f92a9b"
  - 
    x: 6.9
    y: 4.7
    theta: 90.0
    name: "waitingArea"
    uuid: "4109424b-20d0-5707-869c-dc62fea9a658"
  - 
    x: 12.0
    y: 0.8
    theta: 0.0
    name: "vertex_28"
    uuid: "8c4754ff-8571-5467-9da8-3da0d3ba8caa"
  - 
    x: 12.0
    y: 2.4
    theta: 0.0
    name: "vertex_29"
    uuid: "d30fd462-a84d-5940-b533-3f486e0b8561"
  - 
    x: 12.0
    y: 4.0
    theta: 0.0
    name: "vertex_30"
    uuid: "7d59bcca-aaac-51fc-b3da-3006bae4e835"
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






