The following sections describe how other modules needs to be connected to the SP module. First, the workflow is listed with the repeated image from [SP introduction](./../../admin/SP/opil_desc_SP_intro.md) describing it. Then, each section is dedicated to the computer running the module to which the Central SP is connected.

# Workflow
* Central SP is on the OPIL server
* Topology is calculated in the Central SP from the map file
* Topology update is calculated in the Central SP from the map updates in the Local SP
* There is no service calls implemented yet in firos so map is not transmitted through firos. Each module (Local SP on every AGV, Central SP and HMI) should have its own copy of map file (from CAD or as a SLAM result).
* Map merging is done in the Central SP from map updates of up to three Local SPs (three AGVs) into one global gridmap from which the updated topology is calculated (later it will be from more than three Local SPs, i.e., AGVs)
* HMI should have the initial map file, and ability to present map updates over the initial map 
![SP module architecture](./img/sp.png)


# OPIL server computer with the Central SP

The Central SP on the OPIL server calculates the topology and merges the local map updates from the Local SP on an AGV.

## Topology calculation at the Central SP

To start the calculation of the topology, a _map_server_ needs to be started first, which takes a PNG or PGM file of the map, and then the _maptogridmap_ package, which takes annotations.ini file of the annotations coordinates:
```
terminal 1: roslaunch maptogridmap startmapserver.launch 
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
More detailed explanations and examples can be seen in Section [Topology](opil_api_sp.md#topology).


## <a name="sp">Topology sends updates through firos by integrating received local map updates from the Local SP</a>

For this _mapupdates_ needs to be started on a Local SP. New obstacles are merged and new topology is calculated if _maptogridmap_ is running. An example can be seen in Section [Illustration of topology and map updates](./../../admin/SP/opil_desc_SP_intro.md#topologyupdates).

## Central SP sends the topology through firos for TP and HMI and receives the map updates from the Local SP with ID name robot_0

For sending the topology and receiving the map updates through firos, robots.json and whitelist.json should look like this:
### robots.json
```
{
	"map":{
        "topics": {
            "graph": {
                "msg": "maptogridmap.msg.Graph",
                "type": "subscriber"
            }
        }
    },
	"robot_0":{
		"topics": {
			"newObstacles": {
				"msg": "mapupdates.msg.NewObstacles",
				"type": "publisher"
			}
		}
	}
}
```
### whitelist.json
```
{
    "map": {
        "subscriber": ["graph"],
        "publisher": []
    },
    "robot_0": {
        "subscriber": [],
        "publisher": ["newObstacles"]
    }
}
```
You can find the firos config files in test/config_files/Central_SP_computer.
After putting the json files to firos/config folder run firos as:
```
rosrun firos core.py
```
To test receiving of three robots to Central SP modify the config files accordingly:
### robots.json
```
{
	"map":{
        "topics": {
            "graph": {
                "msg": "maptogridmap.msg.Graph",
                "type": "subscriber"
            }
        }
    },
	"robot_0":{
		"topics": {
			"newObstacles": {
				"msg": "mapupdates.msg.NewObstacles",
				"type": "publisher"
			}
		}
	},
    "robot_1":{
        "topics": {
            "newObstacles": {
                "msg": "mapupdates.msg.NewObstacles",
                "type": "publisher"
            }
        }
    },
    "robot_2":{
        "topics": {
            "newObstacles": {
                "msg": "mapupdates.msg.NewObstacles",
                "type": "publisher"
            }
        }
    }
}
```
### whitelist.json
```
{
    "map": {
        "subscriber": ["graph"],
        "publisher": []
    },
    "robot_0": {
        "subscriber": [],
        "publisher": ["newObstacles"]
    },
    "robot_1": {
        "subscriber": [],
        "publisher": ["newObstacles"]
    }"robot_2": {
        "subscriber": [],
        "publisher": ["newObstacles"]
    }
}
```


# OPIL server computer with the Task Planner - TP

## TP receives the topology through firos

For receiving the topology topics from the Central SP, the packages _maptogridmap_ needs to be in src folder because of the defined ROS messages that will be received through firos.

For receiving the topics through firos, robots.json and whitelist.json should look like this:
### robots.json
```
{
   "map":{
        "topics": {
            "graph": {
                "msg": "maptogridmap.msg.Graph",
                "type": "publisher"
            }
        }
    }
}
```
### whitelist.json
```
{
    "map": {
        "publisher": ["graph"],
        "subscriber": []
    }
}
```
Start firos and write a subscriber for the topics as suggested in Section [Writing a simple listener explaining the maplistener package](opil_api_sp.md#writelis).
You can find the firos config files in test/config_files/TP_HMI_computer.

# OPIL server computer with the Human Machine Interface - HMI

## HMI receives the topology

HMI should have a map file presented visually.

For receiving the topology from the Central SP here is how entities look in OCB (applying in Postman GET localhost:10100/robots):

### Topology entities from the Central SP

```
    {
        "topics": [
            {
                "type": "maptogridmap.msg.Nodes",
                "name": "nodes",
                "structure": {
                    "info": {
                        "origin": {
                            "position": {
                                "y": "float64",
                                "x": "float64",
                                "z": "float64"
                            },
                            "orientation": {
                                "y": "float64",
                                "x": "float64",
                                "z": "float64",
                                "w": "float64"
                            }
                        },
                        "width": "uint32",
                        "map_load_time": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "resolution": "float32",
                        "height": "uint32"
                    },
                    "name": "string[]",
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "x": "float64[]",
                    "y": "float64[]",
                    "theta": "float64[]",
                    "uuid": "string[]"
                },
                "pubsub": "subscriber"
            },
            {
                "type": "maptogridmap.msg.Edges",
                "name": "edges",
                "structure": {
                    "header": {
                        "stamp": {
                            "secs": "int32",
                            "nsecs": "int32"
                        },
                        "frame_id": "string",
                        "seq": "uint32"
                    },
                    "uuid": "string[]",
                    "name": "string[]",
                    "uuid_src": "string[]",
                    "uuid_dest": "string[]"
                },
                "pubsub": "subscriber"
            }
        ],
        "name": "map"
    }
```


