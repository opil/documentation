# Introduction

HMI (Human Machine Interface) is a software layer module of the OPIL architecture. This module’s name was changed to MOD.SW.HMI from AHMI when the requirements changed for the OPIL version 2.0. 
HMI is a web application server with its own local database for storing data needed in this module. HMI serves a web browser user interface for the human agents to monitor and control OPIL data entities. 

### - Image tagging uses semantic versioning.
### - Image tags are identical to the GitLab source repository commit tags.
### - Image tagged latest and biggest version number are identical.
### - Please remove old docker image before pulling, if you have problems updating image.

# Usage of the docker image with docker-compose

**1.  Start middleware**

    - Run Orion Context Broker  using the -corsOrigin switch
        - add "-corsOrigin __ALL"  to the orion startup command
    - ngsiproxy must be started as well, preferably with middleware
        - add ngsiproxy service definition to your middleware docker-compose.yml

An example of middleware docker-compose.yml:
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
    - HMI web app uses its own instance of Mongo DB
        - use different service name than with OCB, i.e. like below "mongodb"
        - db data is backed up in folder ./mongo/data
        - application backs up folder ./public/uploads
    - App is started with initial user configured in environment variables
        - although id or password changed here, user's name and role is always 'admin'
        - password could be changed from UI as well, and should be changed


HMI web app docker-compose.yml:
```
version: "3"
services:
 mongodb:
    image: mongo:3.6
    restart: always
    volumes:
      - ./mongo/data:/data/db
 app:
    image: l4ms/opil.sw.hmi:latest
    environment:
     - inituser=admin
     - initpw=admin
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

    - use public IP address when client web browser is running in different physical machine than HMI
    - login using admin/admin