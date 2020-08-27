# Muraplast
Muraplast is the leading and modern producer of Polyethylene blown film in Croatia and Southeastern Europe. Muraplast needs to optimize the process of taking the products (rolls of PE film) from the production lines, weighting them, and transport from ready to warehouse area while sending the information to the local ERP. This optimization would boost productivity, reduce human error factor in weighting and reduce the manual labor needed to manipulate the products.

## Improvement in manufacturing process needed
Muraplast needs to optimize the process of taking the products (rolls of PE film) from the production lines, weighting them, and transport from ready to warehouse area while sending the information to the local ERP. This optimization would boost productivity, reduce human error factor in weighting and reduce the manual labor needed to manipulate the products (PE film in rolls and can be various dimensions).

## Description of the experiment
The table below outlines current logistic processes and the corresponding process in L4MS



Current Process | Process in L4MS          
------------|:--------------------------
PE film in rolls comes down from unwinder to a pallet.|The Advanced HMI operator initiates a predefined task for “PE roll acquisition and storage”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for production lines, and storage locations. Based on the task specification, OPIL Robotic Agent AGV nodes are assigned to each production line to receive the PE film rolls that comes down from the unwinder of each production line. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task to move AGVs to the unwinder locations.
The shaft on which the roll is winding is removed.|OPIL Human Agent nodes are assigned the task to remove the shaft on which the roll is winding. The OPIL Local HMI Layer informs the human workers that are assigned on the task, for the exact location and nature of the task.
A forklift drives the pallet to a scale.|Once the Human worker informs the OPIL system through the The OPIL Local HMI Layer that the shaft removal task is complete and based on the task specification, the AGVs that are equipped with scale automatically transmit the roll weight to the OPIL Enterprise Applications ERP system through the Cyber-Physical Middleware Layer.
Roll is put on a scale, weighed and its weight is manually written on a label. 
The amount is copied on the paper (production work order).
The roll is taken of the scale and back to pallet. Pallet is driven by a manual fork or fork lift to a special ready to warehouse area. The weight from production work order is copied in the ERP (usually by the end of the shift).|Based on the task specification, roll loaded OPIL Robotic Agent AGV nodes are now assigned the task to navigate to specific storage location in the warehouse to unload the PE roll. Once the roll is unloaded, each OPIL Robotic Agent AGV node becomes available for its next task. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task. The OPIL Task Planner’s Task Supervisor module continuously supervises the task execution.  The Task is continuously monitored through the Task Monitoring and Control module of the OPIL Advanced HMI. Interaction between the OPIL Software System Layer, the OPIL Agent Nodes Layer and the Enterprise Application is achieved through the Cyber-Physical Middleware Layer.


## Main benefits of the Experiments

1. Reduce the occupational health hazard on workers involved in lifting of heavy objects in loading and unloading processes firstly in the pilot site and later in plants worldwide. The automation will remove the wrist, arm and back harm risks.

2. Faster production, more accurate, better resource allocation.

3. Shortage of labour for manual jobs among young people due to increasing trend towards higher education.

4. Interaction between human forklift operators and AGVs

5. Configurability: OPIL utilizes a variable mixture of human forklift operators and AGVs depending on the system’s running availability.

6. Dependability: Unresponsive or faulty AGVs do not influence the completion of the task as complementary resources (other AGVs or human forklift operators are automatically assigned as needed).

## Videos
ICENT Robotics team demonstrated the prototype of Automated Guided Vechicle (AGV) solution, that was tailor-made for Croatian manufacturer Muraplast to solve their intra-factory logistics problem. 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=7koPrwFSvLc
" target="_blank"><img src="http://img.youtube.com/vi/7koPrwFSvLc/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

## Connection of PLC sensor data to OPIL through FIWARE OPC UA agent

In the second round of pilot experiment one of the goals were to visualize the current data of sensors at Muraplast.
21 sensors were monitored that are connected to PLC. Some of the sensors are attached to the blowing machine, such as 15 encoders for cutting the film, the length of the film, and velocity of the line, while four sensors are attached to the wall next to the blowing machine to measure air quality, CO2, humidity and temperature of the air.

### Guide for installing FIWARE OPC UA agent

This guide is a short version of the guide on how to setup the connection to the external OPC UA server:
<https://iotagent-opcua.readthedocs.io/en/latest/user_and_programmers_manual/index.html>.

This guide is tested both in Ubuntu linux and Windows 10.
First, clone the prepared configuration for connection to OPC UA Server:
```
git clone https://github.com/Engineering-Research-and-Development/iotagent-opcua
```


Change the following files:
```
iotagent-opcua/docker-compose-external-server.yml
iotagent-opcua/AGECONF/config.json
iotagent-opcua/AGECONF/config.properties 
```

**config.json** can be created by the MAPPING TOOL started individually and all variables can be renamed afterwards. To start the MAPPING TOOL delete the **config.json** and put comment to line `#configuration=api` in **config.properties**. You should get the **config.json** as the result. Take care that **id** and **type** and variable names do not have special characters as ` ` (blanks), or `:`.



**config.properties** needs to be changed as follows:

1. add the correct IP and port of OPC UA server:
```endpoint=opc.tcp://iotplcsrv:4840```
where **iotplcsrv** is defined in **docker-compose-external-server.yml** as `"iotplcsrv:192.168.0.18"`
2. Uncomment (it is assumed that you started the MAPPING TOOL previously):
`configuration=api`
3. Change header names to desired (will be used as header in subscription):
```fiware-service=opcua_muraplast```, ```fiware-service-path=/demo```
4. To allow subscribing to more than 5 sensors (attributes of the same entity in this case) add:
```
#SubscriptionsStrategy
uniqueSubscription=true
```

The example of used **docker-compose-external-server.yml** is here:
```
version: "3"

services:
  iotage:
    hostname: iotage
    image: iotagent4fiware/iotagent-opcua:latest
    logging:
      driver: "json-file"
      options:
        max-size: "200k"
        max-file: "10"
    networks:
      - hostnet
      - iotnet
    ports:
      - "4001:4001"
      - "4081:8080"
    extra_hosts:
      - "iotplcsrv:192.168.0.18"
    depends_on:
      - iotmongo
      - orion
    volumes:
      - ./AGECONF:/opt/iotagent-opcua/conf
    #  - ./certificates:/opt/iotagent-opcua/certificates

  iotmongo:
    hostname: iotmongo
    image: mongo:3.6
    networks:
      - iotnet
    volumes:
      - iotmongo_data:/data/db
      - iotmongo_conf:/data/configdb

  ################ OCB ################

  orion:
    hostname: orion
    image: fiware/orion:latest
    logging:
      driver: "json-file"
      options:
        max-size: "200k"
        max-file: "10"
    networks:
      - hostnet
      - ocbnet
    ports:
      - "1026:1026"
    depends_on:
      - orion_mongo
    entrypoint: /usr/bin/contextBroker -fg -multiservice -ngsiv1Autocast -statCounters -dbhost mongo

  orion_mongo:
    hostname: orion_mongo
    image: mongo:3.6
    networks:
      ocbnet:
        aliases:
          - mongo
    volumes:
      - orion_mongo_data:/data/db
      - orion_mongo_conf:/data/configdb
    command: --nojournal

volumes:
  iotmongo_data:
  iotmongo_conf:
  orion_mongo_data:
  orion_mongo_conf:

networks:
  hostnet:
  iotnet:
  ocbnet:
```

Since there are 21 sensors coming at 10 Hz it was necessary to limit the log files of standard output to max-size "200k" to prevent the disk usage out of space. The following section is put for each container producing large log files:
```
    logging:
      driver: "json-file"
      options:
        max-size: "200k"
        max-file: "10"
``` 

Start the connection by typing in terminal: 
```
docker-compose -f docker-compose-external-server.yml up -d
```
To close the connection to have a clean restart type:
```
docker-compose -f docker-compose-external-server.yml down -v
``` 
To check if there are data in OCB type the following curl command:
```
curl --location --request GET 'http://localhost:1026/v2/entities/' \
--header 'fiware-service: opcua_muraplast' \
--header 'fiware-servicepath: /demo'
```
where headers need to be equal to the ones put in **config.properties**.
As a response you can have something like:
```
[
    {
        "id": "age01_PLC",
        "type": "ServerInterfaces",
        "Air_quality": {
            "type": "Number",
            "value": 63.98292923,
            "metadata": {
                "ServerTimestamp": {
                    "type": "ISO8601",
                    "value": "2020-04-16T16:42:25.068Z"
                },
                "SourceTimestamp": {
                    "type": "ISO8601",
                    "value": "2020-04-16T16:42:25.068Z"
                }
            }
        },
...etc...
```


### Guide for connection of FIWARE OPC UA agent with QuantumLeap

Prepare the following **docker-compose-quantumleap-cratedb.yml** file:
```
version: "3.5"
services:
  # Quantum Leap is persisting Short Term History to Crate-DB
  quantumleap:
    image: smartsdk/quantumleap:latest
    logging:
      driver: "json-file"
      options:
        max-size: "200k"
        max-file: "10"
    hostname: quantumleap
    container_name: fiware-quantumleap
    ports:
      - "8668:8668"
    depends_on:
      - crate-db
    environment:
      - CRATE_HOST=crate-db

  crate-db:
    image: crate:3.3
    hostname: crate-db
    container_name: db-crate
    ports:
      # Admin UI
      - "4200:4200"
      # Transport protocol
      - "4300:4300"
      - "5432:5432"
    command: crate -Cdiscovery.type=single-node -Clicense.enterprise=false -Cauth.host_based.enabled=false  -Ccluster.name=democluster -Chttp.cors.enabled=true -Chttp.cors.allow-origin="*"
    volumes:
      - crate-db:/data

volumes:
  crate-db: ~
```

Start this file by typing in terminal:
```
docker-compose -f docker-compose-quantumleap-cratedb.yml up -d
```
To have a clean restart type:
```
docker-compose -f docker-compose-quantumleap-cratedb.yml down -v
```
To see if there are any issues or warnings type:
```
docker logs db-crate
```
If you have a warning similar to:
```
[2020-05-05T12:24:10,322][WARN ][o.e.b.BootstrapChecks    ] [Monte Civrari] max virtual memory areas vm.max_map_count [65530] is too low, increase to at least [262144]
```
increase the virtual memory warning by typing in terminal:
```
sysctl -w vm.max_map_count=262144
```
To make the above setting permanent perform the following steps:

1. edit the file /etc/sysctl.conf
2. make entry vm.max_map_count=262144
3. restart

Prepare the following script **post_subscr_PLC.sh** to post subscriptions:
```
curl --location --request POST 'http://localhost:1026/v2/subscriptions/' --header 'fiware-service: opcua_muraplast' --header 'fiware-servicepath: /demo' --header 'Content-Type: application/json' --data-raw '{
  "description": "Notify QuantumLeap of all sensor changes",
  "subject": {
    "entities": [
      {
		"id": "age01_PLC",
        "type": "ServerInterfaces"
      }
    ],
	"condition": { "attrs": [ ] }
  },
  "notification": {
    "http": {
      "url": "http://192.168.18.152:8668/v2/notify"
    },
	"attrs": [ ],
    "metadata": ["dateCreated", "dateModified"]
  }
}'
```
where **id** and **type** is the same as written in **config.json**, and IP address is the address of your computer.

Start the script by typing in terminal:
```
bash post_subscr_PLC.sh
```
Check if data are visible in web page at <http://192.168.18.152:4200>, but change the IP address to your computer's.
