# GERM

## Introduction

GERM (Graphical Entity-relationship Model) is the successor to Predicator I am building. It depends on neo4j, and a few other tools.

GERM stores entity and class information as a series of nodes in a graph, connected by predicates. Predicates are different relationships, with some kind of task-relevant meaning.

There is also a specific format for world information that is associated with GERM, described later in this document. GERM worlds provide simple information for constant relationships between objects that can be used to initialize a graph database. They also provide TF frames and other infromation necessary to compute predicates.

**Why is this not a fork of Predicator?** Mostly because I don't want it to work the same way Predicator does. Predicator is closely tied to the current implementation of CoSTAR; it is used as a blackboard by many parts of the CoSTAR software.

## Packages

### GERM Predicator

This package contains the code to produce predicates from object relationship information. It is written in C++ and uses the Germ ROS messages to communicate to the database.

### GERM Neo4j

The `germ_neo4j` package is the interface to the Neo4j database back-end.

It contains code that creates classes, entities, and predicates in a connected Neo4j database.

### GERM Messages

This package provides the message types we want.

*Object*: defines a single entity in the world, with a name, class, and optional properties.
*Properties*: defines a set of typed key/value pairs for an entity or predicate.
*Predicate*: message type describing a relationship between two entities, an entity and a class, or two classes.
*PredicateInstance*: describes a specific instantiation of a predicate as a relationship between two specific entities or classes.

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

This predicate indicates that *Parent* is to the left of *Child*, from the frame of reference of *Child*.

##### RIGHT-OF

##### ABOVE

##### BELOW

##### IN-FRONT-OF

##### IN-BACK-OF

##### WORLD-LEFT-OF

##### WORLD-RIGHT-OF

##### WORLD-ABOVE

##### WORLD-IN-FRONT-OF

##### WORLD-IN-BACK-OF

##### NEAR

##### NEAR-MESH

##### NEAR-XY

##### TOUCHING

## Conventions

### World Specification

World information is stored in YAML files that can be parsed by the GERM ROS interface in order to initialize a graphical representation of the world. A few of these YAML files will be stored in `germ_ros/world`.

Specification is divided between classes, entities, and predicates.

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


