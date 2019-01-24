# OPIL Cyber Physical MW layer

OPIL is packaged in containers in order to enable a fast and reliable deployment of platform components; 
this approach likewise allows final users to run the OPIL as a smooth and easy process.

Every module of OPIL is therefore released as a **Docker image** publicly available at the 
standard de-facto container content library and ecosystem which is [Docker Hub.](https://www.docker.com/products/docker-hub)

Docker images of OPIL specific modules can be found at [https://hub.docker.com/u/l4ms](https://hub.docker.com/u/l4ms)

Docker images of OPIL generic modules, such as those of which the middleware layer is made of, which are exploited into the Platform, are available at [https://hub.docker.com/u/fiware](https://hub.docker.com/u/fiware)

Depending on the required functionalities, specific modules have to be activated and thus deployed.

The Cyber Physical MW layer, as crucial part of the platform, needs to deployed for each type of OPIL usage/scenario.

## MOD.MW.CM (Context Management)
Context Management module is implemented by **FIWARE GE Orion Context Broker**.
It can be deployed and run very easily by means of [docker-compose](https://docs.docker.com/compose/) tool through which all the dependencies are automatically resolved.

Follow these steps:

1) Install [Docker](https://hub.docker.com/search/?type=edition&offering=community&operating_system=linux) in your system if not present, yet.

2) Install [docker-compose](https://docs.docker.com/compose/install/) in your system if not present, yet.

3) Create a directory on your system (for example OPIL-OCB)

4) Create a new file called **docker-compose.yml** within that directory with the following contents:
  
```python
  mongo:
    image: mongo:3.2
    command: --nojournal
  orion:
    image: fiware/orion
    links:
      - mongo
    ports:
      - "1026:1026"
    command: -dbhost mongo
```
   
5) Execute the following commands:

```bash
  cd <path-of-yml-file>
  docker-compose up -d
  docker-compose ps
```

More details are available at the original FIWARE specific GE [documentation](https://fiware-orion.readthedocs.io/en/1.4.0/user/docker/index.html)

## MOD.MW.BDM (Backend Device Management and protocol adapter)
The Backend Device Management and protcol adapter module is implemented by the [FIWARE GE IDAS](https://catalogue-server.fiware.org/enablers/backend-device-management-idas).
This GE enables to connect IoT devices/gateways to FIWARE based ecosystems by translating IoT specific protocols into the NGSI context information protocol that is the FIWARE standard data exchange model. 
The translation is made by different types of Agents, one for each type of communication protocol.

The specific IDAS agent to be deployed depends on the type of type of sensors used and thus the specific communication protocol adopted.
Currently the following IoT protocols are supported: (through each hyper-link below is possible to get further details as well the installation instructions)

* [IoT Agent for JSON](https://fiware-iotagent-json.readthedocs.io/) – a bridge between HTTP/MQTT messaging (with a JSON payload) and NGSI

* [IoT Agent for LWM2M](https://fiware-iotagent-lwm2m.readthedocs.io/) – a bridge between the Lightweight M2M protocol and NGSI

* [IoT Agent for Ultralight](https://fiware-iotagent-ul.rtfd.io/) – a bridge between HTTP/MQTT messaging (with an UltraLight2.0 payload) and NGSI

* [IoT Agent for LoRaWAN](https://fiware-lorawan.rtfd.io/) – a bridge between the LoRaWAN protocol and NGSI

* [IoT Agent for OPC-UA](https://iotagent-opcua.rtfd.io/) – a bridge between the OPC Unified Architecture protocol and NGSI

For more details please refer to the [FIWARE catalog documentation](https://www.fiware.org/developers/catalogue/) at the IoT dedicated section.

## MOD.MW.WDGT (Advanced Widget mash-up HMI)
Advanced Widget mash-up HMI module is implemented by **FIWARE GE Wirecloud**.
It can be deployed and run by following the instructions at [DockerHub](https://github.com/Wirecloud/docker-wirecloud/#how-to-use-this-image) specific repository.


