# Installation


## RAN VC Setup

Under Connections tab in VC-Router component, set the Host IP to point to the IP of the FCB. Connect VC-Router to every VC-RanLogic component by joining VC-Ranlogic and VC-Router interfaces. Likewise, connect VC- Ranlogic components to their respective AGV and AgvAction components. FCB should be initialized before simulation. First, old entity is deleted if it exists. Then, a new, empty entity is created without attributes. There is a button with this functionality in VC-Router component under Connections & rightarrow; Initialize FCB.

## AGV actions

Currently, VC-RAN supports two actions: "1" is for loading a part and "2" is for unloading it. Action "0" is a dummy action meant for tasks that do not include actions.

## Example demo: VC RAN demo.vcmx

The demo contains two conveyors and one of each of the following components: VC-Router, VC-RanLogic, VC-AgvAction and AGV. There are three tasks which are executed in sequence. First task includes two motion assignments and one action assignment: the AGV moves next to a conveyor through two points and loads a part. Then the AGV moves through two points next to the other conveyor and unloads the part on the conveyor. Finally, the AGV returns to its initial location through three points. This is followed by an empty dummy action to signal the end of the task. The collection of task assignments for the demo is provided as part of a JMeter file.

## Missing features

* Cancelling individual assignments
* Subscribing to FCB entity updates
* Assignment queue from AssignmentStatus message
* Error messages not implemented (not defined)

## Implementation justication and future work options

* Motion and action assignments are collected into dictionaries instead of lists as specified in the [OPIL RAN interfaces documentation](https://gitlab.com/opil_group/documentation/blob/develop/docs/develop/RAN/opil_interfaces_ran.md) because its unknown whether the messages will arrive in right order or if all the messages will arrive within some specified time limit. With this design, messages do not have to be received in the exact order they were sent. Likewise, sending incomplete motion or action tasks to AGV is possible if later thought necessary.

<!---
# Deinstallation

# Upgrades

-->

# Deprecated Features

Currently there are no deprecated features.
