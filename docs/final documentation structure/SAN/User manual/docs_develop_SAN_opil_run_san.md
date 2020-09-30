# Configuration File
This section contains in-depth information about the configuration file that the Getting Started section does not.

The example of generic JSON file for configuration looks like this:
```json
{
    "contextBroker": {
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig": {
        "sensors": [
            {
                "sensorID": "demo_sensor1",
                "operationMode": {
                    "mode": "event-driven"
                },    
                "driver": "DemoDriver",
                "driverConfig": {
                    "ioPin": "18",
                    "sensorType": "ON_OFF_SENSOR",
                    "measurementType": "boolean",
                    "sensorManufacturer": "DemoCompany"
                },
                "sanID": "demo_SAN1"
            }
        ]
    }
}
```

Some additional examples of possible configuration files: 

*A config for a sensor running on the GPIO pin of a RevPi*
```json
{
    "contextBroker":{
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig":{
        "sensors": [
            {
                "sensorId": "sim_1",
                "driver": "GenericRevPi",
                "operationMode":{
                    "mode": "time-series",
                    "broadcastInterval": "15",
                    "measurementInterval": "5"
                },
                "driverConfig": {
                    "ioPin": "I_1",
                    "sensorManufacturer": "Demo Inc.",
                    "sensorType": "Infrared Sensor",
                    "measurementType": "Object Presence"
                },
                "sanId": "SAN_demo"
            }
        ]
    }
}
```


*A config for a sensor running connected by USB (machine agnostic)*
```json
{
    "contextBroker":{
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig":{
        "sensors":
            {
                "sensorId": "usb_1",
                "driver": "exampleUSBsensorDriver",
                "operationMode":{
                    "mode": "event-driven",
                },
                "driverConfig": {
                    "vendorID": "235",
                    "productID": "3321",
                    "validInterfaces": [0],
                    "sensorManufacturer": "Demo Inc.",
                    "sensorType": "Infrared Sensor",
                    "measurementType": "Object Presence"
                }
                "sanId": "SAN_demo"
            }
    }
}
```

The possible attributes of the configuration file are explained here:

Attribute | Expected Value | Description 
----------------|-----------------------|-------------------
host | IP address of Context Broker | The address of the server that you are running the Orion Context Broker server on 
port | Port opened to allow information into the server | The port on which the OCB is listening. Defaults to 1026. 
sensorID | ID of the sensor that is sending data | Every sensor is required to have its own ID. The ID is later used to query data from the OCB 
sensorType | Type of the attached sensor | sensorType is required to specify what exactly sensor is, for example IR sensor, RFID sensor, etc.
measurementType | What type of value sensor is measuring | Required to specify what exactly is being measured. This field is used as a key to facilitate unit detection and conversion; the possible values are: **distance**, **angular**, **temperature**, **weight**, **velocity**, **angularVelocity**, **numeric**, and **boolean**. Where applicable, any input units (set by your driver), will be automatically converted to default units, meters, radians, kelvins, kg, m/s, and rad/s. In case of unsupported Type, it should simply be kept in mind that there is no set default and no conversion will happen.
driver | Name of the driver file | The driver for the sensor/device that processes the incoming signal.
operationMode | mode, (measurementInterval/ broadcastInterval) | sets up for parsing the details of mode. MeasurementInterval is only required for  fixed-interval more, and both measurement and broadcastInterval are required for time-series
mode | event-driven / time-series/ fixed-interval | **Event-driven**: checks for a change in sensor readings at an internally fixed interval, and upon detection, sends the new data to the server; **Fixed-interval**: reads sensor data at a fixed interval (measurementInterval) of time; **Time-series**: takes continous measurements every measurementInterval and saves them until broadcastInterval has elapsed, when the saved measurements are sent to OCB
measurementInterval/broadcastInterval | Time in seconds | An attribute which takes time in seconds as its value. measurementInterval is required for fixed-interval mode, and broadcastInterval should be added in addition for the time-series mode.
ioPin | the GPIO pin used for the sensor | Should be set according to the GPIO pin layout of the used device (used only for GPIO devices)
sanID | Name of running SAN instance | Used as a key further up in the dataflow. Should be the same for all sensors of one SAN instance
vendorID | standard ID of the vendor of the usb device | should be set according to specs of the device **(used only for USB devices)**
productID | standard ID of the device | should be set according to specs of the device **(used only for USB devices)**
validInterfaces | integer list of valid interfaces of device | should be set according to specs of the device **(used only for USB devices)**