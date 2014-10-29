# GERM

## Introduction

GERM (Graphical Entity-relationship Model) is the successor to Predicator I am building. It depends on neo4j, and a few other tools.

GERM stores entity and class information as a series of nodes in a graph, connected by predicates. Predicates are different relationships, with some kind of task-relevant meaning.

There is also a specific format for world information that is associated with GERM, described later in this document. GERM worlds provide simple information for constant relationships between objects that can be used to initialize a graph database. They also provide TF frames and other infromation necessary to compute predicates.

**Why is this not a fork of Predicator?** Mostly because I don't want it to work the same way Predicator does. Predicator is closely tied to the current implementation of CoSTAR; it is used as a blackboard by many parts of the CoSTAR software.

## Packages

### GERM Predicator

This package contains the code to produce predicates from object relationship information. It is written in C++ and uses the Germ ROS messages to communicate to the database.

It reads entity definitions from `./definitions/`, and so it should be started up in the same namespace as other code (by convention, all GERM nodes are grouped together in a `germ` namespace). This node will read two categories of configuration settings: **robots** and **task**. In the future there may be a way to define additional sets of configuration settings. The reason configuration settings are grouped like this is for code reusability: as we add more data describing, say, the robots, we can reuse them across a larger pool of example tasks.

Predicator publishes a series of *PredicateInstanceList* messages, first as a comprehensive list of true/false predicates and subsequently as diffs.

**Note for debugging:** if you're having trouble starting Predicator, make sure you have all of the right parameters set up on the parameter server. Predicator requires object information to be loaded to various description parameters, defined in the world .YAML files, in order to function.

#### Unique Properties

Predicator uses some specific properties, defined for some subset of entities.

  - **tf**: the TF coordinate frame representing the entity's position relative to the world frame.
  - **joint_states_topic**: the ROS topic giving the joint states of the robot/object.
  - **description**: the name of the parameter storing the URDF for the robot.
  - **floating_frame**: if the object is not rigidly attached to the world, we may need to manually keep its position up to date (not sure why). This is just the root link of such objects.

### GERM Neo4j

The `germ_neo4j` package is the interface to the Neo4j database back-end.

It contains code that creates classes, entities, and predicates in a connected Neo4j database.

### GERM Messages

This package provides the message types we want.

  - **Object**: defines a single entity in the world, with a name, class, and optional properties.
  - **Properties**: defines a set of typed key/value pairs for an entity or predicate.
  - **Predicate**: message type describing a relationship between two entities, an entity and a class, or two classes.
  - **PredicateInstance**: describes a specific instantiation of a predicate as a relationship between two specific entities or classes.
  - **PredicateInstanceList**: contains an entire set of instantiated predicates
  - **SourceInfo**: contains information regarding the origin (ROS node) that is producing predicates

### GERM ROS

The package `germ_ros` provides a connection between GERM's Neo4j back and and different application-specific modules via ROS messages. It also contains GERM world definitions and launch files to bring up the GERM ROS interface.

Run this with:

```bash
rosrun germ_ros ros_interface.py __ns:=germ
```

You can change the namespace to whatever you want, or leave it empty. The ROS interface script also takes two other arguments: `_purge:=true` will clear the graph database on startup, and `_address:=XXX` will set the address to use when connecting to the database.

#### Launch Files

There is currently just one launch file set up for GERM. This can be brought up with:

```bash
roslaunch germ_ros peg_task.launch
```

This adds the GERM definitions for a set of entities related to the peg task world, as of 10/24/2014.

## Glossary

This section contains defined entities, classes, and predicates in GERM with some set, expected meaning. Write down conventions here.

### Predicates

Predicates represent some binary truth about the world. They all take the form *(Parent) PREDICATE (Child)*.

##### IS-A

*(Parent) IS-A (Child)* means that the entity named *Parent* is a member of class *Child*. This relationship should not have any additional properties.

##### CONTAINS-PART

*(Parent) CONTAINS-PART (Child) means that *Child* is a specific component of *Parent*. This relationship would describe the relationship between a single link in a robot arm and the arm as a whole, for example.

##### LEFT-OF

This predicate indicates that *Parent* is to the left of *Child*, from the frame of reference of *Child*. This is the positive *x* direction.

##### RIGHT-OF

*Parent* is to the right of *Child*, from local coordinate frame. This is the negative x direction.

##### ABOVE

*Parent* is above *child*. This is the positive *y* direction.

##### BELOW

*Parent* is above *child*. This is the negative *y* direction.

##### IN-FRONT-OF

Positive *z* direction. Note that this is a different set of axes than I used in the world coordinate frame! This is because the WAM arm uses *z* for where each link points, so that makes the most sense for "front" in my mind.

##### IN-BACK-OF

Negative *z* direction. Note that this is a different set of axes than I used in the world coordinate frame! This is because the WAM arm uses *z* for where each link points, so that makes the most sense for "front" in my mind.

##### WORLD-LEFT-OF

Positive *y* direction.

##### WORLD-RIGHT-OF

Negative *y* direction.

##### WORLD-ABOVE

Positive *z* direction.

##### WORLD-IN-FRONT-OF

Positive *y* direction.

##### WORLD-IN-BACK-OF

Positive *z* direction.

##### NEAR

Distance is within some bound, defaulting at 0.20m.

##### NEAR-MESH

Distance as computed by FCL (via MoveIt) within some bound, defaulting at 0.10m.

##### NEAR-XY

Distance in only the world X-Y plain is within some bound, defaulting at 0.10m.

##### TOUCHING

Determines if two objects or links are in collision, as determined by FCL (via MoveIt).

## Conventions

### World Specification

World information is stored in YAML files that can be parsed by the GERM ROS interface in order to initialize a graphical representation of the world. A few of these YAML files will be stored in `germ_ros/world`.

Specification is divided between classes, entities, and predicates.

#### Categories

World specifications are grouped into categories. These categories can be called anything; they are designed to group certain code for reusability. By default, we use two categories:

  - The **robots** category stores information on the robot or robots being used for a task, and other invariant aspects of the world.
  - The **task** category stores information on the task-specific objects. For a constuction task, for example, this would include the different bars and blocks used to build a structure.

Example worlds are stored in `germ_ros/world/`.

#### Entity Properties

Entities have a few specific properties to note.

First, entities often have an associated TF frame. This is stored with the key "tf" and type "tf_frame". This is important because Predicator needs these frames to be able to track geometric relationships about objects.

### Naming Conventions

#### Predicates

Predicates are all upper case and hyphenated:

```
LEFT-OF
HIGHER-THAN
IS-A
```

#### Classes

Classes are camel-case with a leading capital letter:

```
Cup
BarrettArm
```

#### Abstract Entities

Abstract entities represent concepts like "closed" or "open", if this becomes important. They are camel-case with a leading lower-case letter:

```
closedPosition
openPosition
```

#### Entities

Entities are physical objects in the world. These are usually associated with coordinate transforms in ROS.

Entities are underscored and lower case:

```
jons_cup
my_cup
ovaltine_1
```

These naming conventions are not very strongly enforced, though. Entity names are expected to come from third-party object detection modules in many cases.


