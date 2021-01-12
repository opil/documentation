# BPO Installation
You can use the docker container to install the BPO module.

## Install from Docker Container
The BPO docker container is located at [RAMP Docker Registry](https://docker.ramp.eu/?page=1#!taglist/opil/opil.sw.bpo).
Pull the latest docker image of the module using this command:
```
$docker pull docker.ramp.eu/opil/opil.sw.bpo:1.1
```

## How to start the BPO docker container
The Orion Context Broker, the Central SP and the HMI docker containers should be running beforehand. If you haven't yet installed these modules, you can use the installation guides of Central SP and HMI that are available on [RAMP.eu](https://www.ramp.eu/#/documentationOpil). 

It is also necessary to activate the BPO Tab on the HMI module to send the specification required for the BPO setup. To do so, enable the `- task_mgmnt=BPO` command of the environment variable of the hmi service while you set up the docker-compose.yml file of the HMI module.

To install the BPO module, you need to prepare a docker-compose.yml following this example.

First step is to set up a docker-compose.yaml file as follows:

> docker-compose.yaml
```
services: 
  bpo: 
    restart: always
    container_name: bpo
    environment: 
      - "FIWAREHOST=<IP address or hostname of fiware>"
      - "HOST=<IP address or hostname of localhost>"
      - "NETINTERFACE=<netinterface of localhost>"
    image: docker.ramp.eu/opil/opil.sw.bpo:1.1
    network_mode: host
version: "3"
```
The example uses the version 3 and it does not need links to enable services to communicate. It is assumed that for testing the BPO service will be on a local machine (localhost) and OCB, Central SP and HMI are started on another machine. Moreover, `FIWAREHOST` needs to be changed to IP address of the machine running OCB, `HOST` needs to be changed to IP address of the machine running the BPO docker container. `NETINTERFACE` needs to changed to netinterface of that local machine, too. 

Then, go to the directory where you have created the `docker-compose.yaml` file and open a terminal. To start the BPO docker container, run the command below:
```
$docker-compose up
```
or 
```
$docker-compose up -d
```
if you want to run the container in the background.


## Install from source code
Supported $ROS_DISTRO is kinetic.

Clone the BPO repository to your catkin workspace. Then compile it with catkin_make in one folder up.
```
cd <your_catkin_workspace>/src/
git clone https://gitlab.com/opil_group/mod.sw.bpo.git
cd ..
catkin_make
```
To check if everything works as expected follow the [User Manual](User_Manual.md)
