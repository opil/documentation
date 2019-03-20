# TaskSupervisor

The TaskSupervisor is a system monitor that provides information about the ongoing tasks, as well as the general status. The TaskSupervisor is responsible for publishing information about the running tasks as well as those that were stopped (not implemented yet).

The TaskSchedular can be divided into three divisions, described as below: 

* **TaskSchedular**: The TaskSchedular is responsible for creating and monitoring TaskManager. The TaskSchedular denotes a collection of one or more TaskManager.
* **TaskManager**: The TaskManagar is responsible for creating and monitoing Tasks. A TaskManager denotes a collection of one or more Tasks.
* **Task**: A task is the lowest part of the TaskSchedular and describes a transport order that is being executed. Each task includes some information about what will be transported, from where to which destination, how this task will be triggered (e.g. a Sensor Event like a button needs to be pushed) and if there is a follow up/child task.

An example of this TaskSchedular is depicted in the following listing:
```
TaskSchedular
 ->TaskManager_1
	Task_1->Task_2->Task_3
 ->TaskManager_2
	Task_A->Task_B
 ->TaskManager_3
	Task_Foo

```

## TaskLanguage

The *TaskLanguage* is a simple, but powerful approach to describe Intralogistic processes. We are stil looking for a name, if you have a suggestions, feel free to get in touch with us.
The *TaskLanugage* can be divided in 3 different so called __*Primitives*__, which are needed to describe a task: 

- Templates
- Instances
- Tasks

Each of them can be used and described arbitary complex. Comments can be annotated via `#`

### Templates
A *Template* is required, if you want to create later an *instance* for a task. They are the basis, and comparable to a class definition in other languages like Python or C++. A template can contain multiple member variables, like palettes/storage places, multiple conveyer belts, or like a fleet of robots with same capabilities. Therefor, a template summarizes and specifies a set of instances.

You can specify a template like this:

```text
template Position
    position
    type
end

template Sensor
    sensorId
    type
end
```

The template *Position* has two member variables, like *position* and *type*. The template *Sensor* has also two member variables, *sensorId* and *type*. These attributes can be later on accessed inside the Intances. 

It is important that the attributes inside an template begin with a lowercase character. The name has to start with an uppercase character. Each value also needs to be prefixed with four spaces (or an `\t`). 

### Instances
An instance is the actual object of a *Template*. Each *Instance* with the same *Template* does not have to be identical as others. The actual values behind multiple instances also do not have to be from the same type. For example: Given a fleet of robots, which can transport something from A to B. Each robot could transport it differently. A vehicle could drive it to its destination whereby a drone can fly it. As long as they have the same capabilities, they can be categorized into the same template. 

Here are two example Instances of *MyTemplate*

```text
Position moldingPallet
    type = "pallet"
    position = "moldingArea_palletPlace"
end

Position warehouse_pos1
    type = "pallet"
    position = "warehouse_destination_pos"
end

Sensor buttonPalletIsReady
    sensorId = "buttonMoldingArea"
    type = "Boolean"
end
```
The instance *moldingPallet* describes the

Each value also has to be prefixed with four spaces (or an `\t`). In addition to that, each value which was previously specified by a template needs to be set to an actual value. This value can be enclosed by `"`. Also the Instance-Name has to start with a lowercase character. 

### Tasks
A task orchestrates different instances via operations. We do not need to describe who is going to transport an item - it is important that the item will be tansported. It is actually not even important what kind of item is on top or inside a container (in our example case it is a palette). We are just moving a pallet from A to B.
In our example, the tasks *Transport_moldingPallet* will be triggered for example by a sensor event, when a button has been pushed and the value is equal (*== True*). 
It describes tasks which multiple instance have to do, triggers tasks or can be triggered by an event. Their inner commands also need to be prefixed with four spaces (or a `\t`). Currently following operations are parsable: 
`Transport->From->To`, `TriggeredBy`, `OnDone`.


```text
task Refill
    Transport 
    from warehouse_pos1
    to moldingPallet 
end

task Transport_moldingPallet
    Transport
    from moldingPallet
    to warehouse_pos1
    OnDone Refill
    TriggeredBy buttonPalletIsReady.value == True
end
```

A concatenation of *Tasks* is also allowed. In our example, *Transport_moldingPallet* has to be performed first, before the Task Refill will executed. Once the Task *Refill* has been performed, the *Transport_moldingPallet* is going to performed. 

So far all specified tasks will be executed infinite until a new update of a task specification has been done.