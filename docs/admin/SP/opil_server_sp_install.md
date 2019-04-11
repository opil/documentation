There are two ways of installing the SP module. You can use the docker containers or compile it from the source.


# Install from Docker Containers

There are two docker containers - the Central SP and the Local SP.

## The Central SP docker 

The docker container is located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp>

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
        image: l4ms/opil.sw.sp:c2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./annotations.ini:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/annotations.ini:ro
#            - ./map.yaml:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.yaml:ro
#            - ./map.png:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.png:ro
#            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
This example uses the version 3 and it does not need links to enable services to communicate. To update the docker-compose to the working version (1.22) type:
```
sudo curl -L https://github.com/docker/compose/releases/download/1.22.0/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
```
To check the version type:
```
sudo docker-compose --version
```
You should see something similar to:
```
docker-compose version 1.22.0, build f46880fe
```
Then, start it from the folder where you put your docker-compose.yml file:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Central SP.](../start.md#fromdocker)




## The Local SP docker

 The current version of the Local SP contains the Stage simulator. This will be removed soon and will be used from the RAN directly communicating through ROS.

The docker container is located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp>

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
#S&P
    splocal:
        restart: always
        image: l4ms/opil.sw.sp:l2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./local_robot_sim.launch:/root/catkin_ws/src/mod.sw.sp/src/localization_and_mapping/sensing_and_perception/local_robot_sim.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=splocal
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```


# Install from Scratch 

If you are using older version of Firos (v1) use this:

```git clone https://github.com/Ikergune/firos.git```

Right now, all prepared config files are for Firos v1. Update to Firos v2 will come soon.
To install Firos v2:

```git clone --recursive https://github.com/iml130/firos.git```

To change all config files to work with Firos v2 you need to replace some variables in config.json to numbers (check the folowing lines):

```
          "throttling": 3, 
          "subscription_length": 300,
          "subscription_refresh_delay": 0.9
```

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
Sometimes you need to repeat `catkin_make` due to the dependencies between the packages.
To check if everything is working properly follow the guide [Starting from Scratch.](../start.md#fromscratch)



