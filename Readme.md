= GERM

GERM (Graphical Entity-relationship Model) is the successor to Predicator I am building. It depends on neo4j, and a few other tools.

== Why is this not a fork of Predicator?

Mostly because I don't want it to work the same way Predicator does.

== Conventions

=== Predicates

Predicates are all upper case and hyphenated:

'''
LEFT-OF
HIGHER-THAN
IS-A
'''

=== Classes

Classes are camel-case with a leading capital letter:

'''
Cup
BarrettArm
'''

=== Abstract Entities

Abstract entities represent concepts like "closed" or "open", if this becomes important. They are camel-case with a leading lower-case letter:

'''
closedPosition
openPosition
'''

=== Entities

Entities are underscored and lower case:

'''
jons_cup
my_cup
ovaltine_1
'''

This is not very strongly enforced, though.

== Packages

=== GERM Predicator

This package contains the code to produce predicates from object relationship information. It is written in C++.


=== GERM Neo4j

This is the interface to the Neo4j database back-end.

=== GERM Messages


This package provides the message types we want.
