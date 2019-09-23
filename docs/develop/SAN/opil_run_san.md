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
The possible attributes of the configuration file are explained here:

Attribute | Expected Value | Description 
----------------|-----------------------|-------------------
host | IP address of Context Broker | The address of the server that you are running the Orion Context Broker server on 
port | Port opened to allow information into the server | The port on which the OCB is listening. Defaults to 1026. 
sensorID | ID of the sensor that is sending data | Every sensor is required to have its own ID. The ID is later used to query data from the OCB 
sensorType | Type of the attached sensor | sensorType is required to specify what exactly sensor is, for example IR sensor, RFID sensor, etc.
measurementType | What type of value sensor is measuring | Required to specify what exactly is being measured. This field is used as a key to facilitate unit detection and conversion; the possible values are: **distance**, **angular**, **temperature**, **weight**, **velocity**, **angularVelocity**, **numeric**, and **boolean**. Where applicable, any input units (set by your driver), will be automatically converted to default units, meters, radians, kelvins, kg, m/s, and rad/s.
driver | Name of the driver file | The driver for the sensor/device that processes the incoming signal.
operationMode | mode, (measurementInterval/ broadcastInterval) | sets up for parsing the details of mode
mode | event-driven / time-series/ fixed-interval | **Event-driven**: checks for a change in sensor readings at an internally fixed interval, and upon detection, sends the new data to the server; **Fixed-interval**: reads sensor data at a fixed interval of time; **Time-series**: allows for a setting of delay between measurements with broadcastInterval, after which measurements are taken at a specified measurementInterval
measurementInterval/broadcastInterval | Time in seconds | An attribute which takes time in seconds as its value. measurementInterval is required for fixed-interval mode, and broadcastInterval should be added in addition for the time-series mode.
ioPin | the GPIO pin used for the sensor | Should be set according to the GPIO pin layout of the used device (used only for GPIO devices)
sanID | Name of running SAN instance | Used as a key further up in the dataflow. Should be the same for all sensors of one SAN instance
vendorID | standard ID of the vendor of the usb device | should be set according to specs of the device **(used only for USB devices)**
productID | standard ID of the device | should be set according to specs of the device **(used only for USB devices)**
validInterfaces | integer list of valid interfaces of device | should be set according to specs of the device **(used only for USB devices)**