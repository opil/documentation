# SAN Installation

Installing SAN on the machine connected to your sensor can be done in two ways: using the cloned GitLab repository's source code, or simply using a Docker container, which allows you to run SAN with no inconvenience of clicking through source folders and files.

## Using the convenient Docker container

Once you have *docker* installed on the machine connecting your sensor, all that is necessary to get the SAN container from the cloud is to run the following command in a terminal window:

`sudo docker pull l4ms/opil.iot.san:stable`

This will clone the latest stable version of SAN onto your machine.

You can now move on to the getting started section.

## Cloning the GitLab repository

This is not recommended for the end user as it provides no advantages over the Docker container, unless access to the source code is absolutely necessary.

Run `git clone https://gitlab.com/opil_group/mod.iot.san` in a terminal window opened in the folder where you want the source code to be cloned to.

You can now move on to the getting started section.

## Updating SAN

Updating the Docker container simply requires you to run the same command you used to install the SAN container:

`sudo docker pull l4ms/opil.iot.san:stable`

Updating the source code requires a git pull:

`git pull https://gitlab.com/opil_group/mod.iot.san`



# Getting started: SAN Setup using Docker
If you have cloned the source repository, you do not need to create a **docker-compose.yaml** or **Dockerfile** and can move on to editing your existing **config.json** file in `mod.iot.san/src/PythonSAN`.

For Docker setup follow the instructions to create the mandatory files.
## Creating the mandatory files to run SAN in a Docker container

After you have pulled the Docker image from the cloud, you should create a folder to house the necessary files to run and configure SAN.

**If you are running SAN from Docker on a Raspberry Pi, you will need to replace every occurrence of ```l4ms/opil.iot.san:stable``` with ```l4ms/opil.iot.san:stableRPI```**

### docker-compose.yaml
The first file you should create is a **docker-compose.yaml** file. Open it in an editor and enter the following:

```yml
version: '3'
services:
    san:
        build:
            image: l4ms/opil.iot.san:stable
            context: ./
            dockerfile: ./Dockerfile
```

This file will tell Docker where it should look for instructions to configure the container, and where the files needed are.

### Dockerfile
Next you should create a **Dockerfile**. Note that this file does not have a set filetype. Open it in an editor and enter the following:

```
FROM l4ms/opil.iot.san:stable
#COPY statements copy files FROM somewhere on your machine TO somewhere in the 
#docker container. You can add/uncomment statements to copy your drivers and config 
#to the container. In order to add your driver, remove the # from #COPY ./$YOUR_DRIVER.py /code/Drivers and replace $YOUR_DRIVER with how you named your driver.

#COPY  FROM         TO
COPY ./config.json /code
#COPY ./YOUR_DRIVER.py /code/Drivers

#This part checks the requirements for the container. Do not change.
WORKDIR /code
RUN pip3 install -r pip-reqs.txt
RUN apt-get update && apt-get -y install sudo
RUN sudo apt-get -y install usbutils
CMD sudo python3 san.py
```

These are the instructions to Docker on how to configure SAN.

If you are usoing Docker you will need to create your own **config.json** file in the same folder as your **docker-compose.yaml** and **Dockerfile**.
After that, you should copy the following initial, demonstrative version of the file into your created file, and change the **"host"** field to your OCB host address.

### Demonstrative generic config.json file

The example of generic JSON file for configuration looks like this:
```json
{
    "contextBroker": {
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig": {
        "sensors": [
            {
                "sensorID": "demo_sensor1",
                "operationMode": {
                    "mode": "event-driven"
                },    
                "driver": "DemoDriver",
                "driverConfig": {
                    "ioPin": "18",
                    "sensorType": "ON_OFF_SENSOR",
                    "measurementType": "boolean",
                    "sensorManufacturer": "DemoCompany"
                },
                "sanID": "demo_SAN1"
            }
        ]
    }
}
```
**For an in-depth explanation of each possible field in the config.json file, refer to the API Documentation SAN section.** 

## Initialising SAN using cloned git repo

1) Open a Terminal.
2) Navigate to mod.iot.san/installation
3) ./sanInstall.sh
4) Restart terminal

## Running SAN demo

Once you have created your **config.json** file or edited the pre-existing one's **"host"** field to your OCB address, you can now run the SAN demo, which simply sends an alternating boolean signal to the OCB server.

### Running SAN demo with Docker

**If you are running SAN from Docker on a Raspberry Pi, you will need to replace every occurrence of ```l4ms/opil.iot.san:stable``` with ```l4ms/opil.iot.san:stableRPI```**

In order to push the new configuration file that you made to the docker container you will need to run the following command in the terminal opened in the SAN folder:

`sudo docker build -t l4ms/opil.iot.san:stable .`

After that you can run the following command to start SAN in the same terminal window:

`sudo docker run -it --rm --privileged --name="SAN" l4ms/opil.iot.san:stable`

### Running with cloned git repo

Navigate into `mod.iot.san/src/PythonSAN`, and run the following command in the terminal window opened in that folder:

`sudo python3 san.py`

### Stopping SAN
1) Find the terminal where the SAN is running.
2) Press Ctrl+C.
3) SAN will stop all running update threads and automatically clean up dangling information from the OCB server.

## Where to next?

In order to be able to properly interface your sensors to SAN, you will need to write SAN drivers for them. For that, refer to the SAN part of the **API Documentation** section.

# SAN Deinstallation
This explains how to remove SAN.

## Remove *Docker* image

If you have run SAN as a *Docker* image, follow these steps:
 
```docker rm l4ms/opil.iot.san:stable```

## Remove repo downloaded from GIT 

Here, you will have to locate the directory mod.iot.san and remove it manually.

### Updating SAN

Updating the Docker container simply requires you to run the same command you used to install the SAN container:

`sudo docker pull l4ms/opil.iot.san:stable`

Updating the source code requires a git pull:

`git pull https://gitlab.com/opil_group/mod.iot.san`



# SAN Deprecated

Currently there are no deprecated functionalities, however, some of the deprecated components may be found in deprecated folder of the source, but they are mostly for development and debugging reasons.






