# Introduction
 
Welcome to Module's User & Programmers Manual Guide! 

Any feedback on this document is highly welcome, including bug reports, typos or information you think should be included but is not. Please send the feedback through email to: module@l4ms.eu. Thank you in advance.

# Interfaces Produced
 We recommend to read the TaskPlanner/TaskSupervisor introduction to understand our concept of the TaskSupervisor.

## TransportOrderUpdate 
Once, a Transportion by an AGV starts, the TaskPlanner will create a TransportOrderUpdate. 

* id : uuid of the running instance
* type : TransportOrderUpdate
* pickupFrom : string (name of the pickup location)
* deliverTo : string (name of the delivery location)
* name : string (name of this task)
* refMaterialflowUpdateId : id (where has been this transportation defined)
* refOwnerId : id (who has this materialflow/transportation defined)
* taskInfo : int (Idle = 0, WaitForStartTrigger = 1, MovingToPickupDestination = 2, WaitForLoading = 3, MovingToDeliveryDestination = 4, WaitForUnloading = 5)
* updateTime : string (last update of this entity)
* startTime : string (when it has been started)

    
```json
{
    "id": "cb01834e-e34d-42e9-95e5-4b9f461065b3",
    "type": "TransportOrderUpdate",
    "deliverTo": {
      "type": "string",
      "value": "Tag10_11",
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "unicode"
        }
      }
    },
    "name": {
      "type": "string",
      "value": "SupplyTaskFromStorage1ToWorkstation1",
      "metadata": {}
    },
    "pickupFrom": {
      "type": "string",
      "value": "Tag12",
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "unicode"
        }
      }
    },
    "refMaterialflowUpdateId": {
      "type": "string",
      "value": "e01b62e6-efee-11e9-b956-e4b97aae3491",
      "metadata": {}
    },
    "refOwnerId": {
      "type": "string",
      "value": "reviewers hmi",
      "metadata": {}
    },
    "startTime": {
      "type": "string",
      "value": "2019-10-16 10:28:05.288958",
      "metadata": {}
    },
    "state": {
      "type": "string",
      "value": "init",
      "metadata": {}
    },
    "taskInfo": {
      "type": "number",
      "value": 0,
      "metadata": {
        "python": {
          "type": "dataType",
          "value": "int"
        }
      }
    },
    "updateTime": {
      "type": "string",
      "value": "2019-10-16 10:28:05.310872",
      "metadata": {}
    }
  }

```



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
    
    
