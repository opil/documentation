## Creating the mandatory files to run SAN in a Docker container

After you have pulled the Docker image from the cloud, you should create a folder to house the necessary files to run and configure SAN.

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

Next you should create a **Dockerfile**. Note that this file does not have a set filetype. Open it in an editor and enter the following:

```
FROM l4ms/opil.iot.san:stable
#COPY statements copy files FROM somewhere on your machine TO somewhere in the 
#docker container. You can add/uncomment statements to copy your drivers and config 
#to the container. In order to add your driver, remove the # from #COPY ./$YOUR_DRIVER.py /code/Drivers and replace $YOUR_DRIVER with how you named your driver.

#COPY  FROM         TO
COPY ./config.json /code
#COPY ./$YOUR_DRIVER.py /code/Drivers

#This part checks the requirements for the container. Do not change.
WORKDIR /code
RUN pip3 install -r pip-reqs.txt
RUN apt-get update && apt-get -y install sudo
RUN sudo apt-get -y install usbutils
CMD sudo python3 san.py
```

These are the instructions to Docker on how to configure SAN.

Now that you have created these vital files, you can move on to creating the Configuration File **config.json**.
