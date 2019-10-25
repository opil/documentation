# Introduction
 
Welcome to Module's User & Programmers Manual Guide! 

Any feedback on this document is highly welcome, including bug reports, typos or information you think should be included but is not. Please send the feedback through email to: module@l4ms.eu. Thank you in advance.

# Interfaces Produced
 We recommend to read the TaskPlanner/TaskSupervisor introduction to understand our concept of the TaskSupervisor (including the TaskScheduling, TaskManagers and Tasks).

## TaskManager 
The TaskManager is using the parsed output of the specified Task Specification (via TaskLanguage). Once the Task Specification is valid, the set of task are the input for the TaskManager. For example a set of three task has been described:

```
Transport_moldingPallet -> Charge -> Refill
```


The first task which will be executed is the `Transport_moldingPallet`. Once this task is finished, `Charge` is the follow up task. Once `Charge` is finished, the follow up task `Refill` will be executed. As soon the end of the last task of this task-set has been reached, in our case `Refill`, the lifecycle of this TaskManager ends. The TaskSupervisor is responsible to respawn the TaskManager.

 As already mentioned, a TaskManager can have multiple tasks in the queue (taskList) while only one single task is running at the same time. Multiple instances of the taskManager are able to run, identified by their unique ID (UUID).

```json
{
	"id": "60149e8f-3dc0-11e9-8a98-30243282b4ae",
	"type": "taskManager",
	"taskList": {
		"type": "array",
		"value": [{
			"type": "string",
			"value": "Transport_moldingPallet"
		}, {
			"type": "string",
			"value": "Charge"
		}, {
			"type": "string",
			"value": "Refill"
		}],
		"metadata": {}
	},
	"taskManagerName": {
		"type": "string",
		"value": "Transport_moldingPallet",
		"metadata": {}
	},
	"time": {
		"type": "string",
		"value": "2019-03-03 15:26:46.689000",
		"metadata": {}
	}
}
```

*  **taskList**: the list of tasks, which are in included in this taskManager
*  **taskManagerName**: the name of the taskManager
*  **time**: date and time of this entity


## Task 
A task is a running instance of task, which has been defined in the taskManager. 
```json
{
  "id": "fb0a90bf-8200-420c-a0b1-3571fe4c3e85",
  "type": "Task",
  "state": {
    "type": "Number",
    "value": 1,
    "metadata": {}
  },
  "taskManagerId": {
    "type": "string",
    "value": "bba38580-3dbd-11e9-842d-30243282b4ae",
    "metadata": {}
  },
  "taskName": {
    "type": "string",
    "value": "Transport_moldingPallet",
    "metadata": {}
  },
  "time": {
    "type": "string",
    "value": "2019-03-03 15:07:52.421000",
    "metadata": {}
  }
}
```

* **state**: Idle : 0, Running : 1, Waiting : 2, Active : 3, Finished : 4, Aborted : 5, Error : 6
* **taskManagerId**: the ID of the taskManager, who is responsible for spawning this task
* **taskName**: current name of this ongoing task
* **time**: date and time of this entity


## MaterialflowSpecificationState  
This entity provides information about the Materialflow and the processed TaskLanguage.

* id : uuid
* type : MaterialflowSpecificationState
* message : String
* refId: String (reference to the Materialflow Entity)
* state: number (0 == ok, -1 ==  error, >0 == ok with some additional information (tbd))

Example:

```Json
{
	"id": "MaterialflowSpecificationState3fcd7747-7428-44cb-bbfb-2d04509addac",
	"type": "MaterialflowSpecificationState",
	"message": {
		"type": "string",
		"value": "Success",
		"metadata": {}
	},
	"refId": {
		"type": "string",
		"value": "Materialflow1",
		"metadata": {
			"python": {
				"type": "dataType",
				"value": "unicode"
			}
		}
	},
	"state": {
		"type": "number",
		"value": 0,
		"metadata": {
			"python": {
				"type": "dataType",
				"value": "int"
			}
		}
	}
}
```
 

# Interfaces Consumed by the TaskPlanner
 In order to work as designed, the TaskPlanner requires a Materialflow, which is based on the TaskLanguage. This entity is explained below.
## Materialflow

* id : uuid
* type : Materialflow 
* task : Text
* ownerId : Text (Reference to the static UUID of the instance for the HMI)
* active: Boolean (Indidicates, if the Materialflow shall be processed by the TaskSupervisor OR not. Important: ONCE the Hmi shutsdown, the HMI *needs* to set the materialflow to disable. The User needs to enable it manually after an restart of the system. This is required that the system is not doing something unexpected after boot).

Example:

```JSON
{
	"id": "Materialflow1",
	"type": "Materialflow",
	"specification": {
		"value": "template+Position%0A++++position%0A++++type%0Aend%0A%0Atemplate+Sensor%0A++++sensorId%0A++++type%0Aend%0A%0A%23%23%23%23%0A%0ASensor+opticalSensor%0A++++sensorId+%3D+%22optical_sensor1%22%0A++++type+%3D+%22Boolean%22%0Aend%0A%0APosition+moldingPallet%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22Tag10_11%22%0Aend%0A%0APosition+warehouse_pos1%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22Tag12%22%0Aend%0A%0A%23%23%23%23+%0Atask+Transport_Start%0A++++Transport%0A++++from+moldingPallet%0A++++to+warehouse_pos1%0A++++TriggeredBy+opticalSensor.value+%3D%3D+True%0Aend%0A",
		"type": "Text"
	},
	"ownerId": {
		"type": "Text",
		"value": "reviewers hmi"
	},
	"active": {
		"type": "boolean",
		"value": true
	}
}
```
* **TaskSPec**: TaskLanguage specification, forbidden characters inside Orion (comma separated): <, >, ", ', =, ; (, ).
    
    
