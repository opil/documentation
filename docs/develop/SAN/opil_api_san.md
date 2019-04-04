# SAN API and interfacing

## Standard entity format
SAN is connected to HMI (Human Machine Interface) in OPIL, which processes the received data and monitors it/redirects to TP. 
However, because the end result is present in Orion Context Broker, HMI can be replaced by your solution/system. 
Alternatively, you can even replace SAN with your solution, but if you want to link it to HMI,
you will have to keep the entity format what is output by SAN.

First, introduce yourself with how 
[entities and attributes](https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#entity-creation) 
in Orion Context Broker.

Next, observe the example of a generic entity output by SAN:

```json
[
    {
        "id": "SAN_demo",
        "type": "SensorAgent",
      "sensorData": {
        "type": "Array",
        "value": [
            {
                "manufacturer": {
                    "type": "String",
                    "value": "someguy"
                },
                "measurementType": {
                    "type": "String",
                    "value": "Object Presence"
                },
                "readings": {
                    "type": "Array",
                    "value": [
                        {
                            "reading": {
                                "type": "Boolean",
                                "value": false
                            }
                        }
                    ]
                },
                "sensorId": {
                    "type": "String",
                    "value": "sim_1"
                },
                "sensorType": {
                    "type": "String",
                    "value": "Infrared Sensor"
                }
            }
        ]
    }
    }
]
```
Below is the explanation of the entity along with its attributes:

|    Attribute    | Data Type |                                                                                              Description                                                                                             |
|:---------------:|:---------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|    sensorData   |   Array   | sensorData is one of the main attributes and it  corresponds to the particular data regarding sensors  which submitted this data                                                                     |
|     sensorId    |   String  | sensorID attribute contains a unique ID of the particular sensor.  ID will tell the user which sensor exactly has sent the data,  regardless is there are other sensors that are totally the same.   |
|    sensorType   |   String  | sensorType attribute contains the name of the sensor and  determines the type of the sensor.                                                                                                         |
| measurementType |   String  | measurementType contains the information which tells user what has been measured exactly. Sensors of the same type  may measure different things and it is essential to specify.                     |
|   manufacturer  |   String  | manufacturer contains the name of the manufacturer of the sensor                                                                                                                                     |
|     readings    |   Array   | readings is an array of 2 attributes: modifiedTime and reading.  reading corresponds to a particular reading taken  by sensor and modifiedTime specifies the exact time when  the reading was taken. |
|   modifiedTime  |   String  |  modifiedTime specifies the time when the reading was taken by sensor.                                                                                                                               |
*Above: Explanation of attributes of the generic entitiy*

| Attribute |       Data type       |                                                                                                  Description                                                                                                  |
|:---------:|:---------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|  readings | Number/Boolean/String/Array |  reading specified the actual measurement taken by sensor.   It can be a numberic value, a boolean value or a string - it depends   on the particular sensor. From this attribute the actual data is received |

*Above: Explanation of readings attribute*

**!PLEASE NOTE** - the used time format is expected to be ISO8601

## Orion Context Broker API to receive SAN data
You can retrieve the SAN data off the Context Broker using 2 methods: either straight by using HTTP GET API command or
by creating a subscription to SAN entity.

### Using HTTP GET

Use the following command to receive the data
```GET http://orion:1026/v2/entities/SAN_id/```

Substitute orion with the IP address of your Orion Context Broker and SAN_id with your SAN ID (SAN_demo by default)

Do not forget to include the following header: "Accept" - "application/json"

### Using subscription

Please check how 
[subscriptions](https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#subscriptions) are made.

The example of a subscription can look like this:

```POST http://orion:1026/v2/subscriptions```
```json
{
  "description": "A subscription to get info about SAN",
  "subject": {
    "entities": [
      {
        "id": "SAN_demo",
        "type": "SensorAgent"
      }
    ],
    "condition": {
      "attrs": [
        "sensorData",
        "modifiedTime"
      ]
    }
  },
  "notification": {
    "http": {
      "url": "http://yoururl.com"
    },
    "attrs": [
      "sensorData",
      "modifiedTime"
    ]
  },
  "throttling": 2
}
```
*Above: Instead of http://yoururl.com insert the endpoint you want to get data to*

Do not forget to include the following header: "Content-type" - "application/json"
