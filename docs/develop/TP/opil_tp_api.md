# Introduction
 
Welcome to Module's User & Programmers Manual Guide!

Any feedback on this document is highly welcome, including bug reports, typos or stuff you think should be included but is not. Please send the feedback through email to: module@l4ms.eu. Thanks in advance!

# Interfaces Produced
We recommend to read the TaskPlanner/Tasksupervisor introduction to understand our concept of the TaskSupervisor (including the TaskScheduling, TaskManagers and Tasks).

## TaskManager
The TaskManager is using the parsed output of the specified Task Specification (via TaskLanguage). Once the task specification is valid, the set of task are the input for the TaskManager. For example a set of three task has been described:

```
Transport_moldingPallet -> Charge -> Refill
```


The first task which will be executed is the `Transport_moldingPallet`. Once this task is finished, `Charge` is the follow up task. Once `Charge` is finished, the follow up task `Refill` will be executed. As soon the end of the last task of this task set has been reached, in our case `Refill`, the lifecycle of this TaskManager ends. The TaskSupervisor is responsible to respawn the TaskManager.

As already mentioned, a TaskManager can have multiple tasks in the queue (taskList) while only one single task is running at the same time. Multiple instances of the taskManager are able to run, identified by their unique id (UUID).

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
* **taskManagerId**: the id of the taskManager, who is responsible for spawning this task
* **taskName**: current name of this ongoing task
* **time**: date and time of this entity


## TaskSpecState
This entity provides information about the TaskSpec and the processed TaskLanguage.
```json
{
	"id": "TaskSpecState3da0ea66-2067-4a9a-ac6e-641715a3e10a",
	"type": "TaskSpecState",
	"message": {
		"type": "string",
		"value": "Success",
		"metadata": {}
	},
	"refId": {
		"type": "string",
		"value": "TaskSpec1",
		"metadata": {}
	},
	"state": {
		"type": "Number",
		"value": 0,
		"metadata": {}
	}
}
```
* *state*:  Idle : 0, Ok: 1, Error = -1
* *message*: A message according to the state. In case of an error, it will inform you with some more detail information about the error
* *refId*: reference id of the TaskSpec entity

# Interfaces Consumed
In order to work as designed, the TaskPlanner requires a TaskSpecification, which is based on the TaskLanguage. This entity is explained below.
## TaskSpec

```json
{
	"id": "TaskSpec1",
	"type": "TaskSpec",
	"TaskSpec": {
		"type": "Text",
		"value": "template+Position%0A++++position%0A++++type%0Aend%0A%0Atemplate+Sensor%0A++++sensorId%0A++++type%0A++++value%0Aend%0A%0A%23%23%23%23%0A%0APosition+moldingPallet%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22moldingArea_palletPlace%22%0Aend%0A%0APosition+warehouse_pos1%0A++++type+%3D+%22pallet%22%0A++++position+%3D+%22warehouse_destination_pos%22%0Aend%0A%0ASensor+buttonPalletIsReady%0A++++sensorId+%3D+%22buttonMoldingArea%22%0A++++type+%3D+%22Boolean%22%0A++++value+%3D+TRUE%0Aend%0A%0A%23%23%23%23%0A%0A%0Atask+Charge%0A++++Transport+%0A++++from+warehouse_pos1%0A++++to+moldingPallet+%0Aend%0A%0Atask+Test2%0A++++Transport%0A++++from+warehouse_pos1%0A++++to+moldingPallet%0A++++OnDone+Charge%0Aend%0A%0Atask+Refill%0A++++Transport+%0A%09from+warehouse_pos1%0A%09to+moldingPallet%0A++++OnDone+Charge%0Aend%0A%0Atask+Transport_moldingPallet%0A++++Transport%0A++++from+moldingPallet%0A++++to+warehouse_pos1%0A++++TriggeredBy+buttonPalletIsReady.value+%3D%3D+TRUE%0A++++OnDone+Refill%0Aend%0A%0A%0A",
		"metadata": {}
	}
}
```
* **TaskSPec**: TaskLanguage specification, forbidden characters inside Orion (comma separated): <, >, ", ', =, ; (, ).
    
