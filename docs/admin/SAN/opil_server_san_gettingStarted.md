# Start Guide

## Introduction

This section will guide you through the initial configuration of SAN necessary and get you familiarised with the files and methods used to configure SAN.

## Requirements
### Hardware

Current version of SAN fully supports Raspberry Pi model 3B Plus and Kunbus RevolutionPi

Further reading: [Raspberry Pi](https://www.raspberrypi.org/) and [Revolution Pi](https://revolution.kunbus.com/)

Without guarantee, any devices using GPIO to send the data from the sensor to SAN should be compatible as well.

### Software

Before using SAN, make sure you have an instance of Orion Context Broker running. This ensures that SAN has somewhere to connect and send data to.

Installing and running the Human Machine Interaction (HMI) node is strongly recommended in order to more elegantly visualise the data recieved from sensors and integrate with other OPIL modules.

## Where to next?

If you are using Docker to run SAN, move to the Using Docker section. If you are using the source code, move to the Configuration File section.
