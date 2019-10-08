# Local SP Deprecated 
Deprecated features are features that SP still supports but that are not maintained or evolved any longer, or will be used in the future. In particular:

# Firos v1 did not allow arrays of custom ROS messages

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

# Sending larger data on demand - a service mockup
Since there is no service call supported yet in Firos, topic _do_serve_ is used as a service mockup. On another machine that wants to obtain the data, on topic _do_serve_ needs to be sent value "true" or 1. Large data are a) map topic created with _map_server_ from PNG or PGM file and b) gridmap topic created from the map by resampling to cells of size given by the parameter _cell_size_.

## Testing sending map topic on request on machine_1:
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

## Testing sending gridmap topic on request:
To test the sending of gridmap topic the _mux_topics.py_ program needs to be started which sends the gridmap on the whitelisted topic only when master requests.
Start all previous steps (terminal 1, ..., terminal 7), but it works with starting only terminals 1,2,3.
```
terminal 8: rosrun maptogridmap mux_topics.py
terminal 9: rostopic pub /map/do_serve std_msgs/Bool '{data: 1}'
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "map" the topic "realtopology".


## Testing if topics are received on machine 2 through firos
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

#Interconnection between machines

## Towards Orion CB
To send the topics to Orion Context Broker json files needs to be set properly inside the firos/config folder and firos needs to be running.

## Towards Physical IO

This direction is not yet used, meaning that something is being sent to IO from OCB. There are two reasons: a) the map is too big; b) there is no service call so AMCL can not work.

But, in general, to send the topics through firos to Physical IO json files need to be set properly inside the firos/config folder and firos needs to be running.

## firos config json files explained between machine 1 and machine 2

On machine 1 all topics that are being sent through context broker need to be "subscriber", and that are being received from the context broker need to be "publisher". Topics are listed under ids and here we have "map" id and "robot_0" id.

The numbering in names robot_0, robot_1, etc. corresponds to the number of used AGVs and you should correct config files with respect to number of AGVs.

### robots.json
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
### whitelist.json
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

