# GERM

## Introduction

GERM (Graphical Entity-relationship Model) is the successor to Predicator I am building. It depends on neo4j, and a few other tools.

GERM stores entity and class information as a series of nodes in a graph, connected by predicates. Predicates are different relationships, with some kind of task-relevant meaning.

#### Why is this not a fork of Predicator?

Mostly because I don't want it to work the same way Predicator does. Predicator is closely tied to the current implementation of CoSTAR; it is used as a blackboard by many parts of the CoSTAR software.

### Conventions

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

## Packages

### GERM Predicator

This package contains the code to produce predicates from object relationship information. It is written in C++ and uses the Germ ROS messages to communicate to the database.

### GERM Neo4j

The `germ_neo4j` package is the interface to the Neo4j database back-end.

It contains code that creates classes, entities, and predicates in a connected Neo4j database.

### GERM Messages

This package provides the message types we want.

### GERM ROS

The package `germ_ros` provides a connection between GERM's Neo4j back and and different application-specific modules via ROS messages.
