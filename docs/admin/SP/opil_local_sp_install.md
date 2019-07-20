There are two ways of installing the SP module. You can use the docker containers or compile it from the source.


# Install from Docker Container

There are two docker containers - the Central SP and the Local SP.
The Local SP is located at 
<https://hub.docker.com/r/l4ms/opil.iot.sp.local>





## The Local SP as a standalone module


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
To check if everything is working properly follow the guide [Starting from Docker - Local SP.](./opil_local_sp_gettingStarted#fromdockerlocal)

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
To check if everything is working properly follow the guide [How to start the Local SP and RAN docker containers together.](./opil_local_sp_gettingStarted#fromdockerlocalran)




# Install from Scratch 

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
To check if everything is working properly follow the guide [Starting from Scratch.](./opil_local_sp_gettingStarted.md#fromscratch)


