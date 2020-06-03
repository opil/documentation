# Introduction Visual Components - Robot Agent Node (VC RAN)

The VC-RAN (Visual Components Robot Agent Node) is the OPIL IoT Nodes layer module devoted to the control of the virtual robots. VC-RAN provides two main functionalities: it manages virtual robot navigation inside Visual Components simulation software and works as an interface between the virtual robots and the FIWARE Orion Context Broker of the OPIL Cyber Physical Middleware layer.

## Overview VC RAN

Figure 1 presents the structure of VC-RAN. Router handles communication between the Orion Context Broker (OCB) and RanLogic components. Messages are transmitted using signals and interfaces in the simulation layout. For every AGV there is one RanLogic and one AgvAction component. From the OCB-messages received and delivered by the Router, RanLogic filters those that contain the robot id of the AGV. RanLogic stores the data (motion and action assignments) to data structures based on the task id and receive time and handles the data delivery to the AGV (motion assignments) and AgvAction components (action assignments). RanLogic also maintains data structures for completed and cancelled tasks as well as for task variables such as current and last motion and action.

RanLogic sends description message every minute to Router. Current motion message is sent by the AGV every second via RanLogic to Router which delivers both messages to OCB. AGV and AgvAction components send a message after every completed assignment to RanLogic, which the RanLogic uses to update its data structures, task variables and state.


![](./img/VC_RAN_diagram.png){: style="width:80%;display: block;margin-left: auto;margin-right: auto;"}

<center>*Figure 1. Structure of VC-RAN.*</center>

## VC-Ran Logic

### Task handling

Tasks are executed in the order they are received. When a motion or action assignment with a particular task id is first received, it is inserted into a data structure together with timestamp. Collectively, they form the task queue. Received tasks are executed in the order of ascending timestamp value. Robot motion planning and  action execution begins immediately after receiving first assignment message for the task with the lowest timestamp value. All the motion assignments belonging to the same current task are sent to the AGV one by one. After all the motion assignments are completed, action assignments belonging to the current task are sent to the AgvAction component one at a time. The task is completed when all the action assignments have been carried out.

### State machine

RanLogic is always in one of the finite states specified in Figure 2. Starting state is state-waiting, which changes to state-task after new task is received. When the first motion assignment belonging to the current task has been received, the state changes to state-motion and RanLogic will start sending motion assingments to the AGV. After all the motion assignments have been completed and the first action assignment belonging to current task has been received, state changes to state-action. In this state, RanLogic sends action assignments one by one to the AgvAction component until all the assignments are completed and RanLogic returns to state-waiting.

![](./img/VC_RAN_state.png){: style="width:60%;display: block;margin-left: auto;margin-right: auto;"}

<center>*Figure 2. VC-RanLogic state machine.*</center>
