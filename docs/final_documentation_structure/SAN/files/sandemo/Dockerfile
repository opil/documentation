FROM docker.ramp.eu/opil/opil.iot.san:stable
#COPY statements copy files FROM somewhere on your machine TO somewhere in the 
#docker container. You can add/uncomment statements to copy your drivers and config 
#to the container. In order to add your driver, remove the # from #COPY ./$YOUR_DRIVER.py /code/Drivers and replace $YOUR_DRIVER with how you named your driver. For adding your config.rsc, for use on a RevPi, uncomment the corresponding line.

#COPY  FROM         TO
COPY ./config.json /code
COPY ./TrajectoryDriver.py /code/Drivers
COPY ./PulseDriver.py /code/Drivers

#This part checks the slimness of the container. Do not change.
RUN apt-get update && apt-get -y upgrade
RUN apt-get -y autoremove
WORKDIR /code
ENTRYPOINT ["python3", "./san.py"]
