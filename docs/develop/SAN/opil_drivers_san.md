# Drivers for SAN
## Current status
Currently SAN supports plug'n'play facility only for digital sensors due to the data they typically submit.

### Existing drivers and supported devices:
GPIO pin interface on Raspberry Pi 3 and Revolution Pi
Generic USB interface
### How to write your own drivers
A new driver must be:

1) saved in the Drivers directory
2) created as a Python class

this Class must have:

1) a name which is identical to its fileName (except the .py ending)
2) a parameter which is initialized in the constructor named:
    
    - self._last_value

3) methods called:

    - \__init__ -which initialized and otherwise sets up the sensor

    - \__del__ which performs any necessary cleanup when the sensor is done

    - _get_reading_ which gets a reading from a the _current_value_ function and updates the _last_value

    - _has_changed_ which detects if the sensor reading has changed

    - __current_value_ which gets a reading from the sensor


4) any other details and tools it needs to get readings from its sensors

### If the driver lacks some part of these, the system will inform and guide you
