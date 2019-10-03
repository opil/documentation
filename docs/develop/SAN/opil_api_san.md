# SAN API and interfacing
This section will contain information on how SAN sends information to the HMI node, and how you can query and use the available data without HMI.


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
{
    "measurementType": {
        "type": "string",
        "value": "boolean"
    },
    "modifiedTime": {
        "type": "string",
        "value": "2019-07-18T10:23:21Z"
    },
    "readings": {
        "type": "array",
        "value": [
            {
                "type": "SensorReading",
                "value": {
                    "reading": {
                        "type": "boolean",
                        "value": true
                    }
                }
            }
        ]
    },
    "sanID": {
        "type": "string",
        "value": "SAN1"
    },
    "sensorID": {
        "type": "string",
        "value": "sensor7"
    },
    "sensorManufacturer": {
        "type": "string",
        "value": "IDec"
    },
    "sensorType": {
        "type": "string",
        "value": "IR"
    }
}

```
Below is the explanation of the entity along with its attributes:

|        Attribute        | Data Type |                                                                                                                                                                                            Description                                                                                                                                                                                         |
|:---------------:|:---------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|         sensorID        |     String    | sensorID attribute contains a unique ID of the particular sensor.    ID will tell the user which sensor exactly has sent the data, it will be unique for each sensor on the system    |
|        sensorType     |     String    | sensorType attribute contains the type of the sensor e.g. infrared(IR), RFID, etc.                                                                                                                                                                                                |
| measurementType |     String    | measurementType contains the information which tells user what has been measured exactly. Sensors of the same type    may be used for measuring different types of information, hence the need to specify. **For boolean data this should be set to boolean in the configuration file in order to avoid unit conversion conflicts**                                       |
|     manufacturer    |     String    | manufacturer contains the name of the manufacturer of the sensor                                                                                                                                                                                                                                                                         |
|         readings        |     Array     | readings is an array with 1 attribute: reading.    reading corresponds to a particular reading taken    by sensor. reading is an attribute of this readings array because    multiple reading attributes can be batched into the readings array when set by user. |
|     modifiedTime    |     String    |    modifiedTime specifies the time when the reading was taken by sensor.                                                                                                                                                                                                                                                             |


| Attribute |             Data type             |                                                                                                                                                                                                    Description                                                                                                                                                                                                    |
|:---------:|:---------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|    reading | Number/Boolean/String/Array |    reading specified the actual measurement taken by sensor.     It can be a numberic value, a boolean value or a string - it depends     on the particular sensor. From this attribute the actual data is received |

*Above: Explanation of reading attribute*

**!PLEASE NOTE** - the used time format is expected to be ISO8601

## Orion Context Broker API to receive updates on SAN data
You can retrieve the SAN data off the Context Broker using 2 methods: either straight by using HTTP GET API command or
by creating a subscription to a SAN entity.

### Using HTTP GET

Use the following command to receive/GET the data:

unparsed raw output:
```curl http://orion:1026/v2/entities/sensorID -s -S -H 'Accept - application/json'```

SAN-like output:
```curl http://orion:1026/v2/entities/sensorID -s -S -H 'Accept - application/json' | python -mjson.tool```

The entities on the server, after receiving their first updates, would look like this for **two separate sensors**, when called with the SAN-like output GET command:
```json
[
    {
        "id": "optical_sensor1",
        "type": "SensorAgent",
        "measurementType": {
            "type": "string",
            "value": "boolean",
            "metadata": {}
        },
        "modifiedTime": {
            "type": "string",
            "value": "2019-09-11T09:36:53Z",
            "metadata": {}
        },
        "readings": {
            "type": "array",
            "value": [
                {
                    "type": "SensorReading",
                    "value": {
                        "reading": {
                            "type": "boolean",
                            "value": false
                        }
                    }
                }
            ],
            "metadata": {}
        },
        "sanID": {
            "type": "string",
            "value": "SAN1",
            "metadata": {}
        },
        "sensorID": {
            "type": "string",
            "value": "optical_sensor1",
            "metadata": {}
        },
        "sensorManufacturer": {
            "type": "string",
            "value": "LLC",
            "metadata": {}
        },
        "sensorType": {
            "type": "string",
            "value": "ON_OFF_SENSOR",
            "metadata": {}
        },
        "units": {
            "type": "string",
            "value": "boolean",
            "metadata": {}
        }
    },
    {
        "id": "optical_sensor2",
        "type": "SensorAgent",
        "measurementType": {
            "type": "string",
            "value": "boolean",
            "metadata": {}
        },
        "modifiedTime": {
            "type": "string",
            "value": "2019-09-11T09:36:53Z",
            "metadata": {}
        },
        "readings": {
            "type": "array",
            "value": [
                {
                    "type": "SensorReading",
                    "value": {
                        "reading": {
                            "type": "boolean",
                            "value": false
                        }
                    }
                }
            ],
            "metadata": {}
        },
        "sanID": {
            "type": "string",
            "value": "SAN1",
            "metadata": {}
        },
        "sensorID": {
            "type": "string",
            "value": "optical_sensor2",
            "metadata": {}
        },
        "sensorManufacturer": {
            "type": "string",
            "value": "LLC",
            "metadata": {}
        },
        "sensorType": {
            "type": "string",
            "value": "ON_OFF_SENSOR",
            "metadata": {}
        },
        "units": {
            "type": "string",
            "value": "boolean",
            "metadata": {}
        }
    }
]
```

Output with only values (types are ignored in this output):
```curl http://orion:1026/v2/entities/sensorID?options=keyValues -s -S -H 'Accept - application/json' | python -mjson.tool```

*Above: Substitute orion with the IP address of your Orion Context Broker and sensorID with your sensor ID*

### Using subscription

Please check how 
[subscriptions](https://fiware-orion.readthedocs.io/en/master/user/walkthrough_apiv2/index.html#subscriptions) are made.

The example of a subscription POST operation can look like this:

```json
curl -v http://orion:1026/v2/subscriptions -s -S -H 'Content-Type: application/json' -d @- <<EOF
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
                "YOUR_ATTRIBUTE_HERE",
            ]
        }
    },
    "notification": {
        "http": {
            "url": "http://yoururl.com"
        },
        "attrs": [
            "YOUR_ATTRIBUTE_HERE",
        ]
    },
    "throttling": 2
}
EOF
```
*Above: Instead of http://yoururl.com insert the endpoint you want to get data to and substitute orion with the IP address of your Orion Context Broker*

*Available attributes for the "attrs" field can be seen in the generic entity output above*