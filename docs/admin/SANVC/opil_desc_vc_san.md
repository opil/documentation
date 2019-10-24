# Introduction Visual Components Sensor Agent Node
SAN communicates directly with Orion CB using REST API requests. IR, voltage and switch sensors mimic
behavior of real sensors. SAN  supportsc reating entities when simulation is started and updating entities during
simulation. OCB has only one entity per SAN. 

SAN allows to gather data from sensors through component interface and translates component information to JSON format
and updates the entity in OCB. It also creates, updates and deletes entities automatically. 

Currently, updates entity immediately after sensor data is received later it can be possible to send gather more
data and send it periodically.


SAN actuator can track one sensor measurement at a time. The correct sensor and measurement are recognized by the sensor id and the measurement type. 