### What is SAN?

Sensor Agent Node (SAN for short) is one of the modules in IoT layer defined by [OPIL](http://project.l4ms.eu/OPIL-Documentation) architecture. It is responsible for connecting various sensors to OPIL and providing data about them to [Orion Context Broker](https://fiware-orion.readthedocs.io/en/master).

*Figure 1: Simplified working principle of SAN*
![img1.png](docs/admin/SAN/img/img1.png "Simplified working principle")

*Figure 2: Sequence diagram with Raspberry Pi as an example*

![img2.png](docs/admin/SAN/img/img2.png "Raspberry Pi example")

## Current version features:

* Plug'n'play for digital sensors
* Non-complex configuration
* Automatic conversion of non-standard units(standard: meters, radians), and units with prefixes
* Data visualisation using the OPIL Human Machine Interface node
* Three configurable modes for submitting the data: event-driven, time-series, fixed-interval
* Supports Revolution Pi and Raspberry Pi devices
* Supports USB devices

## Where should you start?

In order to start working with SAN you should simply follow the documentation, firstly installing the SAN code or the Docker container following **Installation** and configuring SAN according to **Getting Started**. There is also a very quick starting guide available in the Docker Hub description area. For extended information on **Drivers** and the **Configuration File**, refer to the API documentation SAN section.

