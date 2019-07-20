There are two ways of installing the SP module. You can use the docker containers or compile it from the source.


# Install from Docker Container

There are two docker containers - the Central SP and the Local SP.
The Central SP is located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp.central>

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
        image: l4ms/opil.sw.sp.central:latest
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./annotations.ini:/annotations.ini:ro
            - ./floorplan.yaml:/map.yaml:ro
            - ./floorplan.png:/map.png:ro
            - ./topology.launch:/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
This example uses the version 3 and it does not need links to enable services to communicate. It is assumed that for testing all services will be on the same machine (localhost). If orion is started on another machine, then environment variable of sp needs to have changed FIWAREHOST to IP address of that machine. To update the docker-compose to the working version for the version 3 (1.22) type: (NOTE: you should remove prior versions of docker-compose)
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

First, set up the display for viewing the rviz visualization and Stage:
```
xhost local:root
```
This needs to be called only once.
Then, start it from the folder where you put your docker-compose.yml file:
```
sudo docker-compose up
```
To check if everything is working properly follow the guide [Starting from Docker - Central SP.](./opil_server_sp_gettingStarted#fromdocker)






# Install from Scratch 

To install Firos v2:

```git clone --recursive https://github.com/iml130/firos.git```


Install ROS packages:

```sudo apt-get install ros-kinetic-navigation```

```sudo apt-get install ros-kinetic-gmapping```

```sudo apt-get install ros-kinetic-teleop-twist-keyboard```

Install from SourceCode:

* put all ROS packages of Sensing and Perception to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace in src folder. Then compile it with catkin_make in one folder up.
```
cd ..
catkin_make
```
To check if everything is working properly follow the guide [Starting from Scratch.](./opil_server_sp_gettingStarted.md#fromscratch)


