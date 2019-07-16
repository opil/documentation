There are two ways of installing the SP module. You can use the docker containers or compile it from the source.


# Install from Docker Containers

There are two docker containers - the Central SP and the Local SP.
These two docker containers are located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp>

## The Central SP docker 



To install it you need to prepare a docker-compose.yml following this example:
### <a name="dockercompose">docker-compose.yml</a>

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
#S&P
    sp:
        restart: always
        image: l4ms/opil.sw.sp:2.6-central
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./annotations.ini:/root/catkin_ws/src/maptogridmap/launch/annotations.ini:ro
#            - ./testmap.yaml:/root/catkin_ws/src/maptogridmap/launch/map.yaml:ro
#            - ./testmap.png:/root/catkin_ws/src/maptogridmap/launch/map.png:ro
#            - ./topology.launch:/root/catkin_ws/src/maptogridmap/launch/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
This example uses the version 3 and it does not need links to enable services to communicate. To update the docker-compose to the working version for the version 3 (1.22) type: (NOTE: you should remove prior versions of docker-compose)
```
sudo curl -L https://github.com/docker/compose/releases/download/1.22.0/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
```
and then
```
chmod +x /usr/local/bin/docker-compose
```
To check the version type:
```
sudo docker-compose --version
```
You should see:
```
docker-compose version 1.22.0, build f46880fe
```

Then, start it from the folder where you put your docker-compose.yml file:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Central SP.](./opil_server_sp_gettingStarted#fromdocker)




## The Local SP docker as a standalone module


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
        image: l4ms/opil.sw.sp:2.6-local
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./testmap.yaml:/root/catkin_ws/src/localization_and_mapping/lam_simulator/yaml/map.yaml:ro
#            - ./testmap.png:/root/catkin_ws/src/localization_and_mapping/lam_simulator/yaml/map.png:ro
#            - ./amcl_testmap.launch:/root/catkin_ws/src/localization_and_mapping/lam_simulator/launch/amcl_map.launch:ro
#            - ./testmap.world:/root/catkin_ws/src/localization_and_mapping/lam_simulator/yaml/map.world:ro
#            - ./local_robot_sim.launch:/root/catkin_ws/src/localization_and_mapping/sensing_and_perception/local_robot_sim.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
            - SIMULATION=true
```
It is important to put the environment variable `SIMULATION=true`.
Then, start it from the folder where you put your docker-compose.yml file:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Local SP.](./opil_server_sp_gettingStarted#fromdockerlocal)

## The Local SP docker working with RAN

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
        image: l4ms/opil.sw.sp:2.6-local
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./testmap.yaml:/root/catkin_ws/src/localization_and_mapping/lam_simulator/yaml/map.yaml:ro
#            - ./testmap.png:/root/catkin_ws/src/localization_and_mapping/lam_simulator/yaml/map.png:ro
#            - ./amcl_testmap.launch:/root/catkin_ws/src/localization_and_mapping/lam_simulator/launch/amcl_map.launch:ro
#            - ./local_robot.launch:/root/catkin_ws/src/localization_and_mapping/sensing_and_perception/local_robot.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
            - SIMULATION=false
```
Then, start it from the folder where you put your docker-compose.yml file:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
To check if everything is working properly follow the guide [How to start the Local SP and RAN docker containers together.](./opil_server_sp_gettingStarted#fromdockerlocalran)




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
To check if everything is working properly follow the guide [Starting from Scratch.](./opil_server_sp_gettingStarted.md#fromscratch)


