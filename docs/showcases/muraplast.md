# Muraplast
Muraplast is the leading and modern producer of Polyethylene blown film in Croatia and Southeastern Europe. Muraplast need to optimize the process of taking the products (rolls of PE film) from the production lines, weighting them, and transport to ready to warehouse area while sending the information to the local ERP. This optimization would boost production productivity, reduce human error factor in weighting and reduce the manual labor needed to manipulate the products.

## Improvement in manufacturing process needed
MP need to optimize the process of taking the products (rolls of PE film) from the production lines, weighting them, and transport to ready to warehouse area while sending the information to the local ERP. This optimization would boost production productivity, reduce human error factor in weighting and reduce the manual labor needed to manipulate the products. Products are PE film in rolls and can be various dimensions.

## Description of the experiment
The table below outlines current logistic processes and the corresponding process in L4MS



Current Process | Process in L4MS          
------------|:--------------------------
PE film in rolls comes down from unwinder to a pallet. The shaft on which the roll is winding is removed. A forklift drives the pallet to a scale. Roll is put on a scale, weighed and its weight is manually written on a label. The amount is copied on the paper (production work order).|The Advanced HMI operator initiates a predefined task for “PE roll acquisition and storage”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for production lines, and storage locations. Based on the task specification, OPIL Robotic Agent AGV nodes are assigned to each production line to receive the PE film rolls that comes down from the unwinder of each production line. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task to move AGVs to the unwinder locations. OPIL Human Agent nodes are assigned the task to remove the shaft on which the roll is winding. The OPIL Local HMI Layer informs the human workers that are assigned on the task, for the exact location and nature of the task. Once the Human worker informs the OPIL system through the The OPIL Local HMI Layer that the shaft removal task is complete and based on the task specification, the AGVs that are equipped with scale automatically transmit the roll weight to the OPIL Enterprise Applications ERP system through the Cyber-Physical Middleware Layer.
The roll is taken of the scale and back to pallet. Pallet is driven by a manual fork or fork lift to a special ready to warehouse area. The weight from production work order is copied in the ERP (usually by the end of the shift).|Based on the task specification, roll loaded OPIL Robotic Agent AGV nodes are now assigned the task to navigate to specific storage location in the warehouse to unload the PE roll. Once the roll is unloaded, each OPIL Robotic Agent AGV node becomes available for its next task. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task. The OPIL Task Planner’s Task Supervisor module continuously supervises the task execution.  The Task is continuously monitored through the Task Monitoring and Control module of the OPIL Advanced HMI. Interaction between the OPIL Software System Layer, the OPIL Agent Nodes Layer and the Enterprise Application is achieved through the Cyber-Physical Middleware Layer.


## Main benefits of the Experiments

1. Reduce the occupational health hazard on workers involved in lifting of heavy objects in loading and unloading processes firstly in the pilot site and later in plants worldwide. The automation will remove the wrist, arm and back harm risks.

2. Faster production, more accurate, better resource allocation.

3. Shortage of labour for manual jobs among young people due to increasing trend towards higher education.

4. Interaction between human forklift operators and AGVs

5. Configurability: OPIL utilizes a variable mixture of human forklift operators and AGVs depending on the system’s running availability.

6. Dependability: Unresponsive of faulty AGVs do not influence the completion of the task as complementary resources (other AGVs or human forklift operators are automatically assigned as needed).

## Videos
ICENT Robotics team demonstrated the prototype of Automated Guided Vechicle (AGV) solution, that was tailor-made for Croatian manufacturer Muraplast to solve their intra-factory logistics problem. 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=YOUTUBE_VIDEO_ID_HERE
" target="_blank"><img src="http://img.youtube.com/vi/7koPrwFSvLc/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>



