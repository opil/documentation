# Chemi-Pharm Case

Chemi-pharm's main business is manufacturing and sales of high-quality disinfectants and cleaning agents, targeted mainly at the medical sector. Currently Chemi-Pharm is in the processes of dveloping its in-house IT system that would link together all different departments, functions and processes. This will in many ways create the basis for future automation, in regards to knowledge, information and data management. It also helps to reduce the amount of time spent on making entries into IT systems, it will reduce the amount of human errors and make planning and use of resources more efficient.

## Improvement in manufacturing process needed

Currently Chemi-Pharm is in the processes of developing its in-house IT system that would link together all different departments, functions and processes. This will in many ways create the basis for future automation, in regards to knowledge, information and data management. It also helps to reduce the amount of time spent on making entries into IT systems, it will reduce the amount of human errors and make planning and use of resources more efficient.

## Description of the experiment
The desired level of automation is to achieve ultimate flexibility in terms of manufacturing time and product variety and taking as many people out of the processes on the base level as possible. In near-future it would be desirable to get rid of non-value adding activities through automation

|**Current Process** | **Process in L4MS**|
|------------|:--------------------------|
|Raw materials on pallets are unloaded from trucks by warehouse workers using forklifts. | When the truck with raw material arrives, the Advanced HMI operator initiates a predefined task for “raw material unloading, quantity check and storage”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for quantity check locations, storage locations and the number of Human Agent Nodes and Robotics Agent AGV nodes that need to be mobilized. Based on the task specification, OPIL Human Agent nodes that operate forklift and Robotics Agent AGV nodes are assigned the task to pick up pallets from the truck and unload them at specific quantity check locations. At the same time Human Agent Nodes that are quantity checkers are assigned to check the quantities on pallets.
|Products are checked for quantity by warehouse workers and expected delivery article in the IT system is confirmed (tablet computer), which automatically prints a sticker with patch number and QR code for the items, that is put on every pallet. |The OPIL Local HMI Layer informs the human quantity checkers of all the parameters of the quantity checking task and utilizing information from the OPIL Enterprise Applications automatically prints a sticker with patch number and QR code for the items, that is put on every pallet.
|Warehouse workers move the pallets to a suitable storing location in the warehouse, using a forklift or an electric stacker, and make an inventory movement entry using tablets (scanning the QR code or selecting the items manually). | For every pallet that is quantity checked the OPIL Task Planner automatically assigns an available Human or Robotic AGV Agent with the task to move the pallet to a specific storage location in the warehouse. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task. The OPIL Task Planner’s Task Supervisor module continuously supervises the task execution.  The Task is continuously monitored through the Task Monitoring and Control module of the OPIL Advanced HMI. Interaction between the OPIL Software System Layer, the OPIL Agent Nodes Layer and the Enterprise Application is achieved through the Cyber-Physical Middleware Layer.|
|Production manager creates a work order for mixing a product and sends it to the mixing worker. | The Advanced HMI operator (production manager) initiates a predefined task for “product A mixing”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for material locations, mixing location, process timing and valid mixing sequences.
|Mixing worker walks to the ingredients, finds the right ingredient and brings a specified ingredient to the product mixer, using a forklift or an electric stacker. |Concurrent “product XYZ mixing” tasks are combined and optimized across shared resources utilizing information from the OPIL Enterprise Applications. Based on the task specification, OPIL Robotics Agent AGV nodes are assigned the task to pick the correct raw material and deliver it to the mixing work cell(s) on the correct time.
|Mixing worker scans the QR code with the tablet to confirm the patch number of the ingredient and make sure it is the right ingredient. He weighs the ingredient and pours it into the mixer. Then he takes the ingredient container back to its storage area and repeats the process with all the necessary ingredients. | A Human Agent mixing worker is assigned to the mixing work cell and the OPIL Local HMI Layer informs the human worker of all the necessary task details. OPIL Robotics Agent AGV nodes return the material to the storage area when no active mixing operation requires it. Based on the Task Specification, the OPIL Task Planner ensures that depleted raw mixing material is promptly replaced by appropriately assigning a corresponding motion task to an available Robotic Agent AGV.

## Main benefits of the Experiments

1. Automate the transporting and moving of raw materials and ready products. A variety of chemicals with various container sizes needs to be transported from warehouse to the mixers depending on the product under manufacturing. The crates of the finished products can vary in terms of weight and size and should be transported to storage before the delivery.

2. Reduce human factor and effort in terms of making entries into IT systems to bare minimum. The quality control is fundamental for our customer, any error can lead to significant loses. The logistics system shall automate the recording of raw materials and finished products.

3. Interaction between human forklift operators and AGVs

4. Configurability: OPIL utilizes a variable mixture of human forklift operators and AGVs depending on the system’s running availability.

5. Dependability: Unresponsive of faulty AGVs do not influence the completion of the task as complementary resources (other AGVs or human forklift operators are automatically assigned as needed).

## Videos
The following video, an AGV in the StageRos simulation environment has been addressed by an preliminary TaskPlanner. The map is based on Chemi-Pharm AS blueprints.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=vHUJEozs-KE
" target="_blank"><img src="http://img.youtube.com/vi/vHUJEozs-KE/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

This video represents the simulation of integration of OPIL v1 into ChemiPharm manufacturing cell with the use of Visual Components software.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=rBJ2doOHFto
" target="_blank"><img src="http://img.youtube.com/vi/rBJ2doOHFto/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

## OPIL Deployment
To be Done


