# Engino Case
Engino® Toy System helps pupils build technological models creatively and easily so that they can experiment and learn about science and technology in a Playful way. As Engino is growing rapidly, they have a lot of waste in human labour for the transfer of materials from one production area to another. Even at the current state of the company they could save 1 full time person by automating some of the process.

## Improvement in manufacturing process needed
As Engino® Toy System  is growing rapidly, we see that we have a lot of waste in human labour for the transfer of materials from one production area to another. Even at the current state of the company we could save 1 full time person by automating some of the process. As we grow and number of products increase, the waste in people’s time would be exponentially higher.
Another serious problem is the idle time of the machines while waiting for replenishments. This can be reduced by having people bringing materials right before those are needed but as we have many items this requires high level of automation with Warehouse Management System (WMS). 
Finally, they are the mistakes as humans can pick the wrong material and by the time we find out this may go into production leading to rejection production batches. If it is identified early then you have the process of returning the material and pick up a new one. This causes waste, which we have not quantified.

## Description of the experiment
The table below outlines current logistic processes and the corresponding process in L4MS.


Current Process | Process in L4MS          
------------|:--------------------------
The injection machines produce different plastic toy parts. 
The production goes into carton boxes and is sealed. 
We put a sticker with the name and a person takes them to the warehouse. 
From there another person takes the boxes and puts them in the warehouse on different shelf positions accordingly.|The Advanced HMI operator (production manager) initiates a predefined task for “part acquisition and storage”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for part production times, production machine number and locations and storage locations. 
Based on the task specification, Human Agent worker(s) are assigned with the task of sealing and putting stickers on the boxes. All the necessary task data is available to the human workers through the  OPIL Local HMI Layer. 
Based on the task parameters, available OPIL Robotic Agent AGV nodes are mobilized to pick the boxes and place them at specific locations in the warehouse. OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task. 
The OPIL Task Planner’s Task Supervisor module continuously supervises the task execution. The Task is continuously monitored through the Task Monitoring and Control module of the OPIL Advanced HMI. 
Interaction between the OPIL Software System Layer, the OPIL Agent Nodes Layer and the Enterprise Application is achieved through the Cyber-Physical Middleware Layer.
|Then, another (or same) person, picks up the case with the parts that are needed for the packaging machine (which has 8 hoppers and counts parts and seals in bags) and delivers those near the machine. 
The operator feeds the machine and whenever there is a need for replenishment or new parts then someone will go and bring them, sometimes even the operator who has to stop the machine and replenish.|The Advanced HMI operator (production manager) initiates a predefined task for “part packaging for toy A”. The task is parameterized and optimized utilizing information from the OPIL Enterprise Applications, for the necessary parts for toy A, storage locations, packaging machine location, and storage locations. Concurrent “part packaging for toy XYZ” tasks are combined and optimized across shared resources utilizing information from the OPIL Enterprise Applications. 
Based on the task specification, OPIL Robotics Agent AGV nodes are assigned the task to pick the correct raw material and deliver it to the packaging work cell(s). Based on the task specification, Human Agent worker(s) are assigned with the task of the packaging machine operation and all the necessary task data is available to the human workers through the  OPIL Local HMI Layer. 
Based on the task parameters, available OPIL Robotic Agent AGV nodes are mobilized to pick the appropriate box from the storage location and deliver it to the packaging machine location. OPIL Robotics Agent AGV nodes return the material to the storage area when no active packaging operation requires it. 
Based on the task specification, the OPIL Task Planner ensures that depleted packaging material is promptly replaced by appropriately assigning a corresponding motion task to an available Robotic Agent AGV, ensuring no dead-times. 
OPIL IoT Agent sensor nodes and OPIL Robotic Agent AGV nodes provide the necessary information for localization and mapping to the OPIL Sensing and Perception module, while the OPIL Local Execution Layer of the AGVs implements the motion planning task. The OPIL Task Planner’s Task Supervisor module continuously supervises the task execution. 
The Task is continuously monitored through the Task Monitoring and Control module of the OPIL Advanced HMI. Interaction between the OPIL Software System Layer, the OPIL Agent Nodes Layer and the Enterprise Application is achieved through the Cyber-Physical Middleware Layer.

## Main benefits of the Experiments

1. Improved productivity by reducing the waste associated with transport of parts and products.

2. Reduce production error by automating the raw material transport.

3. Improve quality control by reducing the packaging errors when final boxes are filled.

4. Interaction between human packaging machine operators and AGVs.

5. Configurability and adaptability: OPIL seamlessly scales up with the number of production machines and the number of AGVs.

6. Dependability: Unresponsive of faulty AGVs do not influence the completion of the task as complementary resources (other AGVs) are automatically assigned as needed.

