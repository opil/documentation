# HMI Installation

HMI (Human Machine Interface) is a software layer module of the OPIL architecture. This module’s name was changed to MOD.SW.HMI from AHMI when the requirements changed for the OPIL version 2.0. 
HMI is a web application server with its own local database for storing data needed in this module. HMI serves a web browser user interface for the human agents to monitor and control OPIL data entities. 

<!---
 - Image tagging uses semantic versioning. Major release 3 is done according to agreement we should do so.
 - Image tags are identical to the GitLab source repository commit tags.
 - Image tagged latest and biggest version number are identical.
 - Please remove old docker image before pulling, if you have problems updating image. And destroy unnecessary containers as well.
 - Please NOTICE: From version 2.0.20-beta you _must_ configure OCB and NGSI Proxy environment variables _only_ in docker-compose.yml (System Settings -tab is removed from the UI).
-->


## Usage of the docker image with docker-compose

**1.  Start middleware**

    - Run Orion Context Broker  using the -corsOrigin switch
        - add "-corsOrigin __ALL"  to the orion startup command
    - ngsiproxy must be started as well, preferably with middleware
        - add ngsiproxy service definition to your middleware docker-compose.yml

An example of middleware ***docker-compose.yml***:
```
version: "3"
services:
 mongo:
   image: mongo:3.4
   command: --nojournal
 orion:
   image: fiware/orion
   depends_on:
     - mongo
   ports:
     - "1026:1026"
   command: -dbhost mongo -corsOrigin __ALL
 ngsiproxy:
   image: fiware/ngsiproxy
   ports:
     - "3000:3000"
```

**2. Start HMI web application**

    - HMI docker-compose service could be run in the same machine as OCB
        - if so, combine services from YML below to the upper one
        - make necessary changes to environment variables
        - uncomment optional variables if needed
    - HMI web app uses its own instance of Mongo DB
        - use different service name than with OCB, i.e. like below "mongodb"
        - db data is backed up in folder ./mongo/data
        - application backs up folder ./public/uploads
    - App is started with initial user configured in environment variables
        - although id or password changed here, user's name and role is always 'admin'
        - password could be changed from UI as well, and should be changed


HMI web app ***docker-compose.yml***:
```
version: "3"
services:
 mongodb:
    image: mongo:3.6
    restart: always
    volumes:
      - ./mongo/data:/data/db
 app:
    image: ramp-dreg.eurodyn.com/opil/opil.sw.hmi:latest
    environment:
     - inituser=admin
     - initpw=admin
     - ocb_host={IP address or hostname of OCB}
     - ocb_port=1026
     - ngsi_proxy_host={IP address or hostname of NGSI Proxy}
     - ngsi_proxy_port=3000
     #- link_btn_txt={Text to be shown on the button}
     #- link_btn_url={URL to be opened}
     #- task_mgmnt=BPO
    restart: always
    volumes:
      - ./public/uploads:/usr/src/app/public/uploads
    ports:
      - "80:8081"
    depends_on:
      - mongodb
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






