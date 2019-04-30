# Overview

The module Sensing and Perception (SP) is a software module as part of OPIL (Open Platform for Innovation in Logistics). 
It provides the pose of the AGV inside the built map of the environment in which the AGV is navigating and updates the map with the new sensor readings.
Additionally, it can build the map with SLAM (Simultaneous Localization And Mapping) if no map is given initially. It uses the laser scan data for map building and updating the map, and odometry sensors (encoders, IMU) together with lasers and map for localization.

Link to other modules of OPIL (opil-MODULENAME.l4ms.eu):

* [Robot Agent Node (RAN)](http://opil-ran.l4ms.eu)
* [Human Agent Node (HAN)](http://opil-han.l4ms.eu)
* [Sensor Agent Node (SAN)](http://opil-san.l4ms.eu) 
* [Task Planner (TP)](http://opil-tp.l4ms.eu)
* [Human Machine Interface (HMI)](http://opil-hmi.l4ms.eu)
* [Context Management (CM)](http://opil-cm.l4ms.eu) 

There are two instances of SP module: 

* Central SP
* Local SP

Every AGV has it's own Local SP, that takes care of localization and mapping. It localizes the AGV inside the map, and it creates a local map of AGV's surrounding.
Every AGV sends this local map data as an update to a Central SP, which is on the OPIL server. The Local SP is connected to Robot Agent Node (RAN) through ROS Master at RAN side so that navigation inside RAN can get the pose data at high rate.  
The Central SP creates a topology map of the factory floorplan for the Task Planner (TP) and Human Machine Interface (HMI) using the local updates from AGVs.
The following figure explains the architecture of the SP module with the given initial map of the factory (either after a SLAM process or extracted from a CAD drawing). Orange boxes are modules developed for SP, while blue ones are the standard ROS modules. Required inputs are map file with its parameter (PNG file), file with annotations (loading, unloading areas, etc.), laser data (SCAN) and odometry data (ODOM).

![SP module architecture](./img/sp.png)

The functionalities of the SP modules are listed as follows:

## Localization - the Local SP

* Calculates a pose with covariance of the AGV inside the built map
* Calculates a pose with covariance of the AGV inside the incrementally built map of the SLAM process
* Sends a pose with covariance of the AGV to Task Planner, HMI, RAN

## Mapping - the Local SP

* Creates a map with SLAM
* Uses a map from CAD or as result of SLAM for localization
* Creates data for map updates from new laser reading of previously unoccupied areas

## Mapping - the Central SP

* Uses a map from CAD or as result of SLAM for localization
* Creates a topology for Task Planner from the map and annotations file
* Merges local map updates from the Local SP into a global map (gridmap) and updates the topology

## <a name="topologyupdates">Illustration of localization, topology and map updates</a>

An example of SP module functionalities in a built map used in the demo in Zagreb at the review meeting is shown in the following figures. 


### Annotations and topology
This figure shows the topology creation with the annotations marked with yellow arrows (the loading, unloading and waiting areas).
Blue squares are nodes of the topology graph, while lines connecting them are edges of the topology graph. Red squares are the gird cells which contain obstacle within the square of specified size (1.5 m in this example).
From the annotation file containing the annotation coordinates, distance and theta, it is calculated where the AGV needs to be placed in front of the annotation according to the distance from the annotation and the orientation theta. It changes the values of the computed nodes from gridmap cells so that Task Planner can use these nodes as goals.
![Annotations and topology](./img/annotationswithannotations.png)

### <a name="mapupdates1">Map updates 1</a>
The next figure shows the map updates and the localization within the known map. Green dots are the laser readings sensed from the calculated pose (x,y,theta) marked with red arrow.
Tiny red squares (0.1 m) are the local map built with the laser readings. 

![Map updates 1](./img/mapupdates1.png)

### Map updates 2 - change of topology
To illustrate how local map updates influence on the topology, a box was moving in front of the robot. There are more data in the local map updates (tiny red squares) which are used in the Central SP for topology update. Here, it can be seen that one node became occupied and is removed from the topology graph. The edges that are connected to this node are also removed from the topology graph.
![Map updates 2](./img/mapupdates2.png)

### Map updates 3 - change of topology
In this last figure, the new local updates changed the topology even more and one more node and corresponding edges were removed from the topology graph.
![Map updates 3](./img/mapupdates3.png)

## Project layout

    test/                                # The folder with testing files for sending the data with firos
    	docker_compose_files/      		 # The files for testing docker containers
    		Central_SP_docker/           # The files for testing Central SP docker container
    			docker-compose.yml       # The docker-compose file for starting the Central SP container
    			annotations.ini          # The file with annotation coordinates
    			CHEMI.yaml          	 # The file with map parameters (resolution)
    			map.png                  # The map file
    			topology.launch          # The launch file where you can set the size of the node (grid cell)
    	config_files/                    # The configuration files with firos json entities
    		Central_SP_computer/		 # The configuration files for Central_SP_computer that receives the map updates and calculates the topology
    			config.json		         # Change IP and interface for the Central_SP_computer
    			robotdescriptions.json	 # Always the same
    			robots.json		         # Defined what will be sent: nodes, edges; and received: map updates
    			whitelist.json		     # Also as above
    		Local_SP_computer/			 # The configuration files for Local_SP_computer that sends the map updates and pose
    			config.json		         # Change IP and interface for the Local_SP_computer
    			robotdescriptions.json	 # Always the same
    			robots.json		         # Defined what will be sent: map updates and pose
    			whitelist.json		     # Also as above
    		TP_HMI_computer/			 # The configuration files for TP_HMI_computer that receives the topology and pose
    			config.json		         # Change IP and interface for the TP_HMI_computer
    			robotdescriptions.json	 # Always the same
    			robots.json		         # Defined what will be received: topology and pose
    			whitelist.json		     # Also as above
    		machine_1/                   # The configuration files for machine_1 that sends the topology and pose
    			config.json              # Change IP and interface for the machine_1 and machine_2
    			robotdescriptions.json   # Always the same
    			robots.json              # Defined what will be sent: topology and pose; and received: do_serve
    			whitelist.json		     # Also as above
    		machine_2/			         # The configuration files for machine_2 that receives the topology and pose
    			config.json		         # Change IP and interface for the machine_2
    			robotdescriptions.json	 # Always the same
    			robots.json		         # Defined what will be sent: do_serve; and received: topology and pose
    			whitelist.json		     # Also as above
    src/                                 # The source code that needs to be put inside the catkin workspace
        localization_and_mapping/        # The ROS metapackage for localization and SLAM
        	andymark_driver/		     # Characteristic for Anda robot developed at ICENT
        	firos_config/                # Config files for sending the pose
        	husky/                       # Drivers used by Anda robot
        	odometry_correction/         # Improvement of odometry with IMU sensor for Anda robot
        	lam_simulator/               # Simulations of world files in stage simulator
        		launch/                  # Launch files for MURAPLAST, ICENT and IML floorplans
        	sensing_and_perception/      # The ROS package that creates a pose message to be send through firos
        maptogridmap/                    # The ROS package for topology creation and merging map updates
        mapupdates/                      # The ROS package for calculating map updates from the laser readings
        maplistener/                     # The ROS package for visualization of ROS topics

# Next steps

Follow the [Install instructions](./install/install.md) and then the [Start guide.](./start.md)   
For more details how everything works check the [API.](./user/api.md)     
        
