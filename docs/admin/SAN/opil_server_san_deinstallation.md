# Deinstallation
This section will tell you the methods on how to remove SAN.

## Remove *Docker* image
Docker provides tools to remove anything unwanted using command line.

Run `sudo docker system prune` to get rid of anything unused that is not associated with a container.
Run `sudo docker rmi l4ms/opil.iot.san:stable` to remove the SAN image if it was missed by the previous command.

## Remove repo downloaded from GIT 
Here, you will have to locate the directory mod.iot.san and remove it manually.

