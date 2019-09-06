## Using a cloned GitLab repository for SAN

If you have a cloned repository, you already have a **config.json** file. Navigate to mod.iot.san/src/PythonSAN and you should see it. Open it in an editor to configure.

## Using the Docker container for SAN

You will need to create your own **config.json** file in the same folder as your **docker-compose.yaml** and **Dockerfile**.
After that, you should copy the following initial, demonstrative version of the configuration file.

### Demonstrative generic config.json file

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
measurementType | What type of value sensor is measuring | Required to specify what exactly is being measured. For example distance, weight, etc. **For boolean measurements the "measurementType" should be set to "boolean", otherwise SAN will raise a unit conversion error**
driver | Name of the driver file | The driver for the sensor/device that processes the incoming signal.
operationMode | mode, (measurementInterval/ broadcastInterval) | sets up for parsing the details of mode
mode | event-driven / time-series/ fixed-interval | Specifies the mode telling the SAN under which circumstances the data must be submitted
measurementInterval/broadcastInterval | Time in seconds | An attribute which takes time in seconds as its value. Required for fixed-interval and time-series operational modes
ioPin | the GPIO pin used for the sensor | Should be set according to the GPIO pin layout of the used device (used only for GPIO devices)
sanID | Name of running SAN instance | Used as a key further up in the dataflow. Should be the same for all sensors of one SAN instance
vendorID | standard ID of the vendor of the usb device | should be set according to specs of the device **(used only for USB devices)**
productID | standard ID of the device | should be set according to specs of the device **(used only for USB devices)**
validInterfaces | integer list of valid interfaces of device | should be set according to specs of the device **(used only for USB devices)** 


*Above: Explanation of attributes of the generic config.json file*

In order to edit the configuration file to work with your OCB server, it is necessary to changes the attribute values in quotation marks (" ") to your own. For example changing the value of *host* to "host": "192.168.1.105".

## Initialising SAN using cloned git repo

1) Open a Terminal.
2) Navigate to mod.iot.san/installation
3) ./sanInstall.sh
4) Restart terminal

## Running SAN demo

If you have set the "host" to your server ip, and correctly copied the example configuration file, you can now run the SAN demo, which simply sends an alternating boolean signal to the OCB server.

### Running with Docker

In order to push the new configuration file that you made to the docker container you will need to run the following command in the terminal opened in the SAN folder:

`sudo docker build -t l4ms/opil.iot.san:stable .`

After that you can run the following command to start SAN in the same terminal window:

`sudo docker run -it --rm --name="SAN" l4ms/opil.iot.san:stable`

### Running with cloned git repo

Navigate into mod.iot.san/src/PythonSAN, and run the following command in the terminal window opened in that folder:

`sudo python3 san.py`

### Stopping SAN
1) Find the terminal where the SAN is running.
2) Press Ctrl+C.
3) SAN will stop all running update threads and automatically clean up dangling information from the OCB server.

# Where to next?

In order to be able to properly interface your sensors to SAN, you will need to write SAN drivers for them. For that, refer to the Introduction to Drivers section.
