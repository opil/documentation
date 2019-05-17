# Installation
SAN is installed through GIT, and then either by using docker, or the installation script.
Additionally, ensure that Orion Context Broker is running and connection with it can be established.

## Docker Demo
In order to run a SAN demonstration using *docker* open theterminal and enter the following command
```docker run -t -e DEMO=true -e CB_HOST="<hostIP>" -e CB_PORT="<port>" l4ms/opil.iot.san```
replace <hostIP> with the ip address of your orion context broker
replace <port> with the port that the orion context broker is listening on

## Using *GIT*

Mind that you will need an access to this repo which currently is provided by OPIL. 

Use the following command to pull the SAN module repository:

```git clone https://gitlab.com/opil_group/mod.iot.san```

Next, go to the mod.iot.san/PythonSAN repository.

## Using *Docker*
In order to run SAN using *Docker* container, open the Terminal, navigate to the docker folder and type the following commands:

```docker-compose up```

ensure that the config file (src/PythonSAN/config.json) is correct
especially, 
ensure that:
-the host and port are the correct host and port
of your Orion Context Broker
-the device to be tested is valid 
(for testing, a USB device can be rapidly configured: go the UserTools directory)


## Upgrading from previous versions
If you are still using SAN ver.1, simply git pull from the most recent version available on gitlab

```git clone https://gitlab.com/opil_group/mod.iot.san```
