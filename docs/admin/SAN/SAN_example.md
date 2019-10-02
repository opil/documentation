*A config for a sensor running on the GPIO pin of a RevPi*
```json
{
    "contextBroker":{
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig":{
        "sensors":
            {
                "sensorId": "sim_1",
                "sensorType": "Infrared Sensor",
                "measurementType": "Object Presence",
                "driver": "GenericRevPi",
                "operationMode":{
                    "mode": "time-series",
                    "broadcastInterval": "15",
                    "measurementInterval": "5"
                },
                "driverConfig": {
                    "ioPin": "I_1"
                },
                "sanId": "SAN_demo"
            }
    }
}
```
- note that the needs of "driverConfig" are listed in *genericRevPi.py* under Drivers

```python
try
    self._ioPin = driver_config['ioPin']
except:
    raise ConfigError('no ioPin specified')
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
                "sensorType": "Infrared Sensor",
                "measurementType": "Object Presence",
                "driver": "exampleUSBsensorDriver",
                "operationMode":{
                    "mode": "event-driven",
                },
                "driverConfig": {
                    "vendorID": "235",
                    "productID": "3321",
                    "validInterfaces": [0]
                }
                "sanId": "SAN_demo"
            }
    }
}
```
- note that the needs of "driverConfig" are listed in *exampleUSBsensorDriver.py* under Drivers

```python
vendorID = driver_config.get('vendorID', None)
if vendorID == None:
raise ConfigError('vendorID must be defined in driver_config')
    productID = driver_config.get('productID', None)
if productID == None:
    raise ConfigError('productID must be defined in driver_config')
validInterfaces = driver_config.get('validInterfaces', None) 
if validInterfaces == None:
    raise ConfigError('validInterfaces must be defined in driver_config')
```
*A config for a both of the above sensors running at once*
```json
{
    "contextBroker":{
        "host": "192.168.0.100",
        "port": "1026"
    },
    "sanConfig":{
        "sensors":
            {
                "sensorId": "sim_1",
                "sensorType": "Infrared Sensor",
                "measurementType": "Object Presence",
                "driver": "GenericRevPi",
                "operationMode":{
                    "mode": "time-series",
                    "broadcastInterval": "15",
                    "measurementInterval": "5"
                },
                "driverConfig": {
                    "ioPin": "I_1"
                },
                "sanId": "SAN_demo"
            },
            {
                "sensorId": "usb_1",
                "sensorType": "Infrared Sensor",
                "measurementType": "Object Presence",
                "driver": "exampleUSBsensorDriver",
                "operationMode":{
                    "mode": "event-driven",
                },
                "driverConfig": {
                    "vendorID": "235",
                    "productID": "3321",
                    "validInterfaces": [0]
                }
                "sanId": "SAN_demo"
            }
    }
}
