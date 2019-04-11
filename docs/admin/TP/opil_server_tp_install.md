#Introduction
 
Welcome to Module's Installation & Administration Guide! 

Any feedback on this document is highly welcome, including bug reports, typos or information you think should be included but is not. Please send the feedback through email to: module@l4ms.eu. Thank you in advance.

# From Scratch
Currently we are recommending *NOT* to install this Task Planner from scratch. We are having a lot of dependencies (ROS, FIROS, Python Packages, ...) therefore we are focussing on the Docker deployment. A installation from scratch will follow soon.

# How to use the TaskPlanner with Docker

 This module depends on FIWARE's Orion Context Broker. We will include a setup for Orion only inside the Docker-Compose file. If you are following Steps 2 and 3, you have to include Orion by yourself.

## 1. The Fastest Way
Docker Compose allows you to set up the OPIL TaskPlanner in a few minutes. For this, you have to have Docker-Compose installed. 


Follow these steps:

 1. Create a directory on your system on which to work (for example, ~/opil). 

2. Create a new file called docker-compose.yml inside your directory with the following contents:
    ```
    version: "3.1"

    services:

        #Database for context broker
        mongo:
            restart: always
            image: mongo:3.4
            command: --nojournal                  

        #Context Broker
        orion:
            restart: always
            image: fiware/orion
            links:
                - mongo
            ports:
                - 1026:1026
            command: 
                -dbhost mongo -corsOrigin __ALL  
                
        #Task Planner
        tp:
            image: l4ms/opil.sw.tp:latest
            depends_on:
                - orion
            ports:
                - 2906:2906
            volumes:
                - ./fiware_config.ini:/app/taskplanner/fiware_config.ini  
    ```
 

3. Using the command-line and within the directory you created type: sudo docker-compose up. 

Regarding --nojournal it is not recommened for production, but it speeds up mongo container start up and avoids some race conditions problems if Orion container is faster and doesn't find the DB up and ready. 

After a few seconds you should have your OPIL TaskPlanner running and listening on port `2906`.


Check that everything works with

`curl localhost:2906`


The expected output should be a:

```
Fraunhofer IML
OPIL TaskPlanner VERSION_NUMBER - DATE
Running...
```


What you have done with this method is download images for OPIL TaskPlanner, Orion Context Broker and MongoDB from the public repository of images called Docker Hub. Then you have created two containers based on both images.

 If you want to stop the scenario you have to press Control+C on the terminal where docker-compose is running. Note that you will lose any data that was being used in Orion using this method.

## 2. Run one container

, this way doesn't include a running instance of FIWARE's Orion Context Broker and a database like MongoDb. 


`sudo docker run -it -p 2906:2906-v absolutPathConfigFile:/app/taskplanner/fiware_config.ini l4ms/opil.sw.tp:latest`

```sudo docker run -it -p 2906:2906-v /home/oe130/dockertest/taskplanner/Docker/fiware_config.ini:/app/taskplanner/fiware_config.ini l4ms/opil.sw.tp:latest```

Check that everything works with

```curl localhost:2906```

The expected output should be a:

```
Fraunhofer IML
OPIL TaskPlanner VERSION_NUMBER - DATE
Running...
```

## 3. Build with a docker file

 by creating your own image, you will have more control and a deeper understanding of what is going on inside the Task Planner. We are recommding to do it this way, only if you are familiar with Docker Images and how to create them. Probably, it will be never necessary to create this Docker image on your own, since we have built it for you. However, if you want to do so, please follow these steps: 

Steps: 

1. Download TaskPlanner's source code from Github (git clone <URL>)
 2. cd mod.sw.tp/docker
 3. Modify the Dockerfile to your liking
 4. Run TaskPlanner...

    1. `sudo docker build -t opil.sw.tp` .
    2. `sudo docker run -it -p 2906:2906-v absolutPathConfigFile:/app/taskplanner/fiware_config.ini opil.sw.tp`

      
Check that everything works with

`curl localhost:2906`

The expected output should be a:

```
Fraunhofer IML
OPIL TaskPlanner VERSION_NUMBER - DATE
Running...
```

The parameter -t opil.sw.tp in the docker build command gives the image a name. This name could be anything, or even include an organization like -t l4ms/module-tp. This name is later used to run the container based on the image. 

If you want to know more about images and the building process you can find it in Docker's documentation.


## 4. Fiware Config File
```ini
[flask]
host = 0.0.0.0 

[taskplanner]
; hostname or ip address of the TaskPlanner machine 
; Please note that "tp" in docker-compose.yml must match
host = tp
; Port of the task planner
PORT = 2906

[contextbroker]
; hostname or ip address of the context broker machine 
; Please note that "orion" in docker-compose.yml must match
host= orion
; Port of the context broker
port=1026
```

# Other info

## 5. Other Information: 

Things to keep in mind while working with docker containers and OPIL's TaskPlanner.

### 5.1 Data persistence: once you are closing the docker container, data will be non-persistent.

### 5.2 Subscriptions: close the docker container by using the `Ctrl-C` command. While we are receiving this command, we are unsubscribing from the Orion Context Broker. If you close this container improperly, some old subscription may mess up the behaviour.