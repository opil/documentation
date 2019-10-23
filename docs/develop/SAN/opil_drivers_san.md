# Drivers for SAN

## Existing drivers and supported devices:
GPIO pin interface on Raspberry Pis and Revolution Pi
Generic USB interface

**Driver for Revolution Pi boolean ioPin sensor**
Requires you to set the ioPin from which the value will be read and will return a True value when the pin is read to be False.
```python
import SANDriver
from time import sleep
class RevPiDigital(SANDriver.SANDriver):
    def setup(self):
        self.GPIO = self.importIO('revpimodio2')
        self._ioPin = self.fromConfig('ioPin')
        self.revpi = self.GPIO.RevPiModIO(autorefresh=True, configrsc="config.rsc")
        self.setMeta('sensorManufacturer', self.fromConfig('sensorManufacturer'))
        self.setMeta('measurementType', self.fromConfig('measurementType'))
        self.setMeta('sensorType', self.fromConfig('sensorType'))


    def get_reading(self):
        return self.revpi.io[self._ioPin].value == False
```


**Driver for Raspberry Pi 3 boolean ioPin sensor**
Requires you to set the ioPin from which the value will be read and will return a True value when the pin is read to be LOW by GPIO.
```python
import SANDriver
from time import sleep

class RPiDigital(SANDriver.SANDriver):

    def setup(self):
        GPIO = self.importIO('RPi.GPIO')
        self.ioPin = int(self.fromConfig('ioPin'))
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ioPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        self.GPIO = GPIO
        self.setMeta('sensorManufacturer', self.fromConfig('sensorManufacturer'))
        self.setMeta('measurementType', self.fromConfig('measurementType'))
        self.setMeta('sensorType', self.fromConfig('sensorType'))

    def get_reading(self):
        return self.GPIO.input(self.ioPin) == self.GPIO.HIGH
```
## How to write your own drivers

Drivers are Python files that get called by SAN to tell the main program how and from where to interpret the sensor's data, as well as read some **config.json** information to be sent to the OCB server later.

Note that SAN is using [Pint](https://pint.readthedocs.io/en/0.9/ "Pint: makes units easy") for unit conversion, so any distance unit and prefix supported by [Pint](https://pint.readthedocs.io/en/0.9/ "Pint: makes units easy") will be automatically supported by SAN as well, as long as you properly set your reading units. For rotational units, degrees and radians are supported at the moment.

**We will use the Raspberry Pi 3 boolean ioPin sensor driver as example**
```python
import SANDriver
from time import sleep

#Every driver needs a unique name to later be called from the config.json, this name should also be the name of the .py file.
#In this case the name is RPiDigital. Keep the content in the parentheses unchanged.
class RPiDigital(SANDriver.SANDriver):

    #The following is a setup function. This gets run only once, when SAN is starting.
    #This function is a requirement in any driver.
    def setup(self):
        #Importing GPIO
        GPIO = self.importIO('RPi.GPIO')

        #Taking the pin number to forward to the GPIO.
        self.ioPin = int(self.fromConfig('ioPin'))

        #Setting up the corresponding pin to default to a DOWN (0, non-excited) value
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ioPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        self.GPIO = GPIO

        #Reading necessary data from the configuration file
        self.setMeta('sensorManufacturer', self.fromConfig('sensorManufacturer'))
        self.setMeta('measurementType', self.fromConfig('measurementType'))
        self.setMeta('sensorType', self.fromConfig('sensorType'))
        #In case your data will be sent to the server in a non-boolean form, you will also need to set the units for the data
        #Setting units is done by adding the following line to the code:
        #self.setUnits("YOUR_UNIT")

    #The following function gets run constantly during SAN operation and will be used to get the value from the pin, and prepare it
    #to be sent to the OCB server
    #This function is a requirement in any driver.
    def get_reading(self):
        #This driver is set up to return (send to the main SAN process) a True value, if the previously set pin detects a LOW value.
        #It will return a False value, if the previously set pin detects a HIGH value.
        
        #This space between the return statement and the calling of the function should be where you do any necessary operations
        #with the data, such as waiting for a time period before sending the value, or double-checking if the value has stayed the
        #same for a time period.

        #Currently SAN only accepts return data if it is either boolean or decimal numeric.

        return self.GPIO.input(self.ioPin) == self.GPIO.HIGH
```

This driver tells SAN to send boolean data from the pin to the server. The boolean value sent will be True, if the pin is detected as GPIO HIGH and False if detected is GPIO LOW.

***Most of the time, if your driver is lacking an important part, SAN will tell you what you need to fix in a terminal message.***

**If you are using the source code, you can see exampleIoPinDriver.py and exampleUSBsensorDriver.py for a similar breakdown of driver construction and access to extra modification tools which should not be necessary under normal SAN operation.**

## Installing your drivers onto the Docker contatiner of SAN

1) You should enter your **Dockerfile** and uncomment the **COPY** statement that should be there, if the example **Dockerfile** was properly copied over. 
2) Replace **YOUR_DRIVER** with your driver filename.
3) Save the file.
4) Rebuild the container by running: `sudo docker build -t l4ms/opil.iot.san:stable .`
5) When that process is done, your drivers have been added to the container's Drivers folder and are accessible by SAN, which can be run with `sudo docker run -it --rm --name="SAN" l4ms/opil.iot.san:stable`

## Adding your drivers to the source repository

You should simply copy your drivers over to `mod.iot.san/src/PythonSAN/Drivers` and they should be accessible by SAN.