# Business Process Optimization
The OPIL BPO (Business Process Optimization) module determines an optimized solution to the optimized motion task sequencing problem. The major focus of BPO are actions that have to be executed by factory floor agents (humans and AGVs) to carry out logistics motion tasks. BPO takes as input a description of the capabilities of the environment, the agents, the cost of the agents’ actions and the expected system state as a task specification in a language specifically developed for this module.


## Getting Started
These instructions is about how to start the BPO docker container given a task specification using the HMI and SP OPIL modules. Agents could be robots, humans, items etc. BPO receives the setup of the environment, the enable transitions from one location to another, the initial state of the agents as the initial location of each agent and a task defined as objective. A task could be "Transport item C at location A". BPO receives these information in a language specifically developed for the needs of this module. Finally, BPO produce the sequence of actions as steps of a shortest path from the initial state of the agents to a target and the time needed.


## How to start the HMI and Central SP docker containers
Before starting the BPO docker container, you should start the [Central SP](https://docker.ramp.eu/?page=1#!taglist/opil/opil.sw.sp.central) and [HMI](https://docker.ramp.eu/?page=1#!taglist/opil/opil.sw.hmi) docker containers. It is also necessary to use the BPO Tab on the HMI module to send the specification required for the BPO setup.


At first, you need to pull the latest docker images of the modules

```
$docker pull docker.ramp.eu/opil/opil.sw.hmi:3.0.18-beta
$docker pull docker.ramp.eu/opil/opil.sw.sp.central:latest
```

Next step is to setup the `docker-compose.yaml` files to run the docker containers. 
In the first `docker-compose.yaml` file, you need to include the Orion Context Broker, the HMI app and the Central SP. You need to add the line to load the BPO tab on the HMI app to send the specially developed specification required for the BPO module. The `docker-compose.yaml` should be:

> docker-compose.yaml
```
version: "3"
services:

#Context Broker
    mongo:
        restart: always
        image: mongo:3.6
        command: --nojournal    

#Proxy for Context Broker
    ngsiproxy:
        image: fiware/ngsiproxy:latest
        depends_on:
            - mongo
        ports:
            - 3000:3000

#Context Broker
    orion:
        image: fiware/orion
        depends_on:
            - mongo
            - ngsiproxy
        ports:
            - 1026:1026
        command:
            -dbhost mongo -corsOrigin __ALL

#Mongo Database
    mongodb:
	  image: mongo:3.6
        restart: always
    	  volumes:
		 - ./mongo/data:/data/db
    hmiapp:
	  container_name: hminode
        image: docker.ramp.eu/opil/opil.sw.hmi:3.0.18-beta
        environment:
		 - inituser=admin
		 - initpw=admin
		 - ocb_host=<IP orion host>
		 - ocb_port=1026
		 - ngsi_proxy_host=<IP ngsiproxy host>
		 - ngsi_proxy_port=3000
		 #- link_btn_txt=Open an External System
		 #- link_btn_url=https://www.cut.ac.cy.com/
		 - task_mgmnt=BPO
	  restart: always
        volumes:
		 - ./public/uploads:/usr/src/app/public/uploads
        ports:
		 - "80:8081"
        depends_on:
		 - mongodb
	  command: bash -c './wait-for mongodb:27017 -- node server.js'

#Central Sensing & Perception
    sp:
        restart: always
        image: docker.ramp.eu/opil/opil.sw.sp.central:latest
        depends_on:
            - orion
        volumes:
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
            - ./annotations.ini:/annotations.ini:ro
            - ./demo.yaml:/map.yaml:ro
            - ./demo.png:/map.png:ro
            - ./topology.launch:/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - DISPLAY=$DISPLAY
        ports: 
            - "39002:39002"
```
                   
In the same folder with the docker-compose.yaml, save your annotations.ini and the map.yam/png files of the layout that the robot, human and items exist.
Open a terminal on the directory where this docker-compose.yaml is located and run the command
```
$docker-compose up
```

Open a web browser, e.g. firefox, to see the entities at the address `http://localhost:1026/v2/entities`. There should be a topic `/map/graph`.


Open another tab on the web browser. 
At the address `http://localhost:1026/main`, you should see the HMI app. Use the `inituser` as username and the `initpw` as password to login the app and go to the BPO tab. 


## How to start the BPO docker container
Pull the latest docker image of the module
```
$docker pull docker.ramp.eu/opil/opil.sw.bpo:1.1
```

Set up the BPO `docker-compose.yaml` the docker container

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

Open a terminal in the directory where the BPO `docker-compose.yaml` is located and run the command
```
$docker-compose up
```


## How to setup the BPO specification
At the address `http://localhost:1026/main`, write the specification in the window of the BPO tab and click the red **"send specification"** button existing under the window. After that, you should see the specification assigned below the window and also the `/BPO/specification` entity at `http://localhost:1026/v2/entities`.

30 seconds later, you should see the topic `bpo/results` at the address `http://localhost:1026/v2/entities`.


## BPO specification explanation
This is an example of the BPO specification send by the HMI app. At the moment, we address a single robot-human-item problem. Multiple robots-humans-items problem could be available soon in the following version of the BPO module.

> BPOSpecificationInput.json
```
{
   "Environment":{
	"locations":{
		"Name":["Warehouse","Injection_machines","Packing_machines","Starting_point_1","Charging_area","anywhere","on_robot_1"],
		"Letter":["A","B","C","D","F","*","R"]
		},
	"robot_1":["A","B","C","D","F"],
	"human_1":["A","B","C","D"],
	"item_1":["A","B","C", "D", "R"]
    },
   "Constraints":{
	"robot_1":{
		"A":["*"],
		"B":["A","C","D"],
		"C":["A","B","D"],
		"D":["A","C","B"],
		"F":["A"]
		},
	"human_1":{
		"A":["B","C","D"],
		"B":"A",
		"C":["A","B"],
		"D":["A","C"]
		},
	"item_1":{
		"A":"R",
		"B":"R",
		"C":"R",
		"D":"R",
		"R":["A","B","C","D"]
		}
    },
   "Starting_point":{
	"robot_1":"F",
	"human_1":"D",
	"item_1":"A"
    },
   "Objective":{
	"item_1":"D"
    }
}
```

`"locations": "Name"` defines the locations in the floor that the robots, humans and items could be. The names should be the same as the ones used in the annotation file as processed by the [opil/opil.sw.sp.central](https://docker.ramp.eu/?page=1#!taglist/opil/opil.sw.sp.central) docker container. `"anywhere"` determines all the defined locations that an agent (robot, human, item) could navigate. The robot agent could transport from location `"A"` to `"B", "C", "D", "F"`. `"on_robot_1"` defines the `"loading"` status when the robot is carrying an item. For the example above, the item can be loaded on the robot (notated as `"R"`) from locations `"A", "B", "C"` 
```
"A":"R", "B":"R", "C":"R"
```
and could be `"unloaded"` from robot to `"A", "B", "C"`.
```
"R":["A","B","C"]
``` 

**Do not omit the `"loading"` or `"unloading"` status transitions, since the item could not be loaded or unloaded from/on the robot. Otherwise, the planner will not be able to find the solution to the given task.**

`"locations": "Letter"` is to set a unique alphabet letter for each predefined location respectively. For the example above, `"Warehouse"` is notated with `"A"`, `"anywhere"` is notated with `"*"` and `"on_robot_1"` is notated with `"R"`. Using the `"Letter"` notations, define the location that an agent is possible to move. For example, the robot agent could be at `"Warehouse"`, `"Injection_machines"`, `"Packing_machines"`, `"Starting_point_1"`, `"Charging_area"`. An item could be at `"Warehouse"`, `"Injection_machines"`, `"Packing_machines"`, `"on_robot_1"`. **Do not forget to include the letter notation for the robot in order to determine that the item is loading on the robot.**

`"Constraints"` enable all the possible transitions from a location to another. For example, the robot agent could transport from location `"A"` to `"B", "C", "D", "F"` or the item can be loaded on the robot (notated as `"R"`) from locations `"A", "B", "C"`.

`"Starting_point"` is the current location of each agent. For example, robot is at `"Charging_area"`, human is at `"Starting_point_1"` and item is at `"Warehouse"` (notated as `"A"`).

Since we have set up the environment, we can now determine the task that BPO is going to solve. The task is the determimnation of the task sequence to the desired location that we would like to transport the item defined in `"Objective"`. For example, 
```
"item_1":"C" 
```
determines that we would like the item to be transported from location `"A"` (current location extracted from the `"Starting_point"`) to location `"D"` (`"Starting_point_1"`).



## BPO results explanation

Using the specification example above, the BPO module seeks for the optimal solution for the given task "Transport item at location C". The BPO result consists of the shortest path and the time needed for the agents to complete the given task. The shortest path determines the less actions that should be performed by the agents (robot, human, item) to complete the given task. Time is the seconds needed to complete the given task. Following the specification example, here is the path explanation.

The BPO output published as topic `/bpo/results` has 2 attributes: 1. `path`, 2. `time`. The path has 7 steps: 
```
FDA, ADA, AAA, AAR, ADR, DDR, DDD
```
Each step is a string composed by letter as many as the agents are. At the moment, each string has 3 letters, one for each agent. The first letter refers to the robot, the second letter refers to the human and the third letter refers to the item. So, user is aware about the current position of the agents at any time during the task execution. Only one change per agent is allowed in every step for the robust control of the process.

- *Step 1*: The starting point of the agents. 
- *Step 2*: Robot is moving first from F to A.
- *Step 3*: Robot reached location A. Now, human is moving from D to A. All the agents are at location A where the item is stored.
- *Step 4*: Human loads the item on the robot at location A.
- *Step 5*: Human goes from A to location D.
- *Step 6*: Robot goes from A to D carrying the item.
- *Step 7*: Human unloads the item from the robot at location D.

`time` is the seconds needed for the agents to perform the actions described in the steps above. In this example, time is 63 seconds.
