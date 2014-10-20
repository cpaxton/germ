#!/usr/bin/env python

from py2neo import neo4j, node, rel

import germ_msgs.msg as gm
import std_msgs.msg as sm

import rospy

class GermDatabaseConnection:

    '''
    addPredicateInstance()
    Process a single instantiated predicate into the system.
    '''
    def addPredicateInstance(self, parent_name, child_name, predicate_name, data):
        entities = self.db.get_or_create_index(neo4j.Node, "Entities")
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")
        predicates = self.db.get_or_create_index(neo4j.Relationship, "Predicates")

        parent = entities.get("name",parent_name)
        child = entities.get("name",child_name)

        if not (len(parent) == 0) and not (len(child) == 0):
            predicates.get_or_create("predicate",parent_name+" "+predicate_name+" "+child_name, (parent[0], predicate_name, child[0]), data)

    '''
    addObject()
    Adds an object with a class.
    '''
    def addObject(self, name, obj_class_name, data):

        entities = self.db.get_or_create_index(neo4j.Node, "Entities")
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")
        predicates = self.db.get_or_create_index(neo4j.Relationship, "ClassPredicates")

        entity = entities.get_or_create("name", name, data)
        entity.add_labels("entity")
        
        obj_class = classes.get_or_create("name", obj_class_name, {"name":obj_class_name})
        obj_class.add_labels("class")

        predicates.get_or_create("predicate",name, (entity, "IS-A", obj_class))

    '''
    addClass()
    Callback to add a single class to the database, as a string,
    '''
    def addClass(self, class_name):
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")

        obj_class = classes.get_or_create("name", class_name)
        obj_class.add_labels("class")

    def __init__(self, db_address="http://localhost:7474/db/data"):
        self.db = neo4j.GraphDatabaseService(db_address)


