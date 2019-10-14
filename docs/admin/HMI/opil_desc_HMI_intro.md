# HMI Introduction

## User management

By installing the HMI module, an user with administrator role is created as well. This user is able to define other users with different user roles. Specific tabs and infomration windows  of the user interface in the web browser can be restricted based on a user’s role. HTTP calls to the HMI web server and certain functions can be restricted as well. All the user data is stored in the local database. Additionally, HMI creates an entity to the OCB for HAN. This entity’s ID is the same unique ID as the user's in the local database.

## Floorplan management

From the user interface it is possible to upload to the system floorplan images in png or jpg formats. User is asked to give a name of the floorplan, scale of the image in centimetres per pixel and x- and y-offset values in relation to the robots map. Currently user interface shows the latest uploaded floorplan of the saved ones.

## Task management

Task specifications at the moment are encoded by the user in written text snippets of the task specification language that are saved in the “TaskSpec” type of entities created to the OCB. Web application then subscribes and listens notifications from OCB’s “TaskSpecState” type of entities. User is able to cancel these task specifications and which entities are then removed from the OCB.

## Viewing robot and sensor status

User interface has its own views for following the robot status and sensor values information. Web application subscribes all type ‘ROBOT’ and type ‘SensorAgent’ entities. Robots view when notified: robot statuses, x- and y-positions and theta value. Sensors show when notified: sensor values in their own chart panel.
