# Running SAN

## Configuration

 Assuming you have downloaded the SAN on Raspberry Pi or Revolution Pi, proceed with the following steps. 

First, go to mod.iot.san/PythonSAN repository and locate the following file - *config.json*

The example of generic JSON file for configuration looks like this:
```json
{
    "contextBroker":{
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig":{
        "sanId": "SAN_demo",
        "sensors": [
            {
                "sensorId": "sim_1",
                "sensorType": "Infrared Sensor",
                "measurementType": "Object Presence",
                "driver": "genericRevPi",
                "operationMode":{
                    "mode": "time-series",
                    "broadcastInterval": "15",
                    "measurementInterval": "5"
                },
                "driverConfig": {
                    "ioPin": "I_1"
                }
            }
        ]
    }
}
```
Below is the explanation of the entity along with its attributes:

|                Attribute                |               Expected value              |                                                      Description                                                     |
|:---------------------------------------:|:-----------------------------------------:|:--------------------------------------------------------------------------------------------------------------------:|
|                   host                  |        IP address of Context Broker       | Expected IP address of the Orion Context Broker server                                                               |
|                   port                  | Port of Context Broker                    | Port on which Context Broker is listening. Default port is 1026                                                      |
|                  sanId                  | ID of the SAN to be executed              | Every SAN system is required to have its own ID. The ID is later used to query data off the Context Broker           |
|                 sensorId                | ID of the attached sensor                 | ID of the sensor is required to distinguish between the sensors of the same type                                     |
|                sensorType               | Type of the attached sensor               | sensorType is required to specify what exactly sensor is, as different sensors may execute identical tasks           |
|             measurementType             | What sensor is measuring                  | Required to specify what exactly is being measured                                                                   |
|                  driver                 | Name of the driver file                   | The actual driver for the sensor/device that processes the incoming signal.                                          |
|                className                | Name of the class that driver is using    | Configures a class used by driver.                                                                                   |
|                   operationMode                  | mode, (measurementInterval/ broadcastInterval) | sets up for parsing the details of mode                              |
|                   mode                  | event-driven/ time-series/ fixed-interval | Specifies the mode telling the SAN under which circumstances the data must be submitted                              |
|  measurementInterval /broadcastInterval | Time in seconds                           | An attribute which takes time in seconds as its value. Required for fixed-interval and time-series operational modes |
|                  ioPin                  | the GPIO pin used for the sensor          | Should be set according to the GPIO pin layout of the used device (used only for GPIO devices)                                                   |
|                  vendorID                 | standard ID of the vendor of the usb device (in hex, ie: 0x1ea7)          | should be set according to specs of the device (used only for USB devices)     |
|                  productID                 | standard ID of the device (in hex, ie: 0x1ea7)          | should be set according to specs of the device (used only for USB devices, necessary only in case of two devices from same vendor)                                                   |
|                 validInterfaces                 | integer list of valid interfaces of device          | should be set according to specs of the device (used only for USB devices, necessary only if the device does not output on interface 0 as is standard)                                         |
|

*Above: Explanation of attributes of the generic config.json file*

You should simply change the value written in quotation marks (" ") according to your value 
(for example, IP address of OCB)

## Installing the SAN
1) Open a Terminal.
2) Navigate to mod.iot.san/installation
3) ./sanInstall.sh
4) Restart terminal

## Running the SAN

 After you have installed the SAN and configured the SAN to run with the necessary sensors and proper IP address of the OCB:
 1) Open a Terminal
 2) Execute the command "san"


```>> san```

## Stopping SAN
Because SAN uses threads to process and submit data, you will have to do the following to stop SAN: 
1) Find the terminal where the SAN is running
2) Type in Ctrl+C
