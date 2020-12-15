# HMI Installation

HMI (Human Machine Interface) is a software layer module of the OPIL architecture. HMI is a web application server with its own local database for storing data needed in this module. HMI serves a web browser user interface for the human agents to monitor and control OPIL data entities. 


## Usage of the docker image with docker-compose

**1.  Start middleware**

    - Run Orion Context Broker  using the -corsOrigin switch
        - add "-corsOrigin __ALL"  to the orion startup command
    - ngsiproxy must be started as well, preferably with middleware
        - add ngsiproxy service definition to your middleware docker-compose.yml

An example of middleware ***docker-compose.yml***:
```yml
version: "3.1"

services:
    ### Database for context broker ###
    mongo:
        image: mongo:3.6
        command: --nojournal

    ### Proxy for Context Broker ###
    ngsiproxy:
        image: fiware/ngsiproxy:1.2
        ports:
            - 3000:3000

    ### Context Broker ###
    orion:
        image: fiware/orion:2.3.0
        depends_on:
            - mongo
            - ngsiproxy
        ports:
            - 1026:1026
        command:
            -dbhost mongo -corsOrigin __ALL -inReqPayloadMaxSize 2097152
```

**2. Start HMI web application**

    - HMI docker-compose service could be run in the same machine as OCB
        - if so, combine services from YML below to the upper one and uncomment the row '- orion' from depends_on
        - make necessary changes to environment variables
        - uncomment optional variables if needed
    - HMI web app uses its own instance of Mongo DB
        - use different service name than with OCB, i.e. like below "mongodb"
        - db data is backed up in volumes ./mongo/data local folder
        - application backs up volumes ./public/uploads local folder
    - App is started with initial user configured in environment variables
        - although id or password changed here, user's name and role is always 'admin'
        - password could be changed from UI as well, and should be changed


HMI web app ***docker-compose.yml***:
```yml
version: "3.1"
services:
    ### Database for HMI ###
    mongodb:
        image: mongo:3.6
        volumes:
        - ./mongo/data:/data/db
    ### HMI web app ###
    hmi:
        image: docker.ramp.eu/opil/opil.sw.hmi:latest
        volumes:
            - ./public/uploads:/usr/src/app/public/uploads
        environment:
            - inituser=admin
            - initpw=admin
            - ocb_host=<ip-address>
            - ocb_port=1026
            - ngsi_proxy_host=<ip-address>
            - ngsi_proxy_port=3000
            - SESSION_SECRET=ChangeThisSecret
            #- link_btn_txt={Text to be shown on the button}
            #- link_btn_url={URL to be opened}
            #- task_mgmnt=BPO
        ports:
            - "80:8081"
        depends_on:
            - mongodb
            #- orion
        command: bash -c './wait-for mongodb:27017 -- node server.js'
```

**3. Open http://hostnameorIP with your web browser**

    - use remote IP address when client web browser is running in different physical machine than HMI
      (not localhost/127.0.0.1)
    - login using admin/admin

# HMI Deinstall
HMI can be deinstalled by removing the Docker container.

# Upgrades
HMI can be upgraded by first shutting down OPIL and the downloading latest Docker image.

Good idea could be to remove the old Docker container and image before.

In HMI ***docker-compose.yml*** is defined the file locations of backed up folders for Mongo DB and uploaded files. If not removed, they will be in use of upgraded HMI app.

# Deprecated Features


Currently there are no deprecated features.






