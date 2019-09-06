# Installation

Installing SAN on the machine connected to your sensor can be done in two ways: using the cloned GitLab repository's source code, or simply using a Docker container, which allows you to run SAN with no inconvenience of clicking through source folders and files.

## Using the convenient Docker container

Once you have *docker* installed on the machine connecting your sensor, all that is necessary to get the SAN container from the cloud is to run the following command in a terminal window:

`sudo docker pull l4ms/opil.iot.san:stable`

This will clone the latest stable version of SAN onto your machine.

You can now move on to the getting started section.

## Cloning the GitLab repository

This is not recommended for the end user as it provides no advantages over the Docker container, unless access to the source code is absolutely necessary.

Run `git clone https://gitlab.com/opil_group/mod.iot.san` in a terminal window opened in the folder where you want the source code to be cloned to.

You can now move on to the getting started section.

## Updating SAN

Updating the Docker container simply requires you to run the same command you used to install the SAN container:

`sudo docker pull l4ms/opil.iot.san:stable`

Updating the source code requires a git pull:

`git pull https://gitlab.com/opil_group/mod.iot.san`
