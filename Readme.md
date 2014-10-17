= GERM

GERM (Graphical Entity-relationship Model) is the successor to Predicator I am building. It depends on neo4j, and a few other tools.

== Why is this not a fork of Predicator?

Mostly because I don't want it to work the same way Predicator does.

== Packages

=== GERM Predicator

This package contains the code to produce predicates from object relationship information. It is written in C++.


=== GERM Neo4j

This is the interface to the Neo4j database back-end.

=== GERM Messages


This package provides the message types we want.
