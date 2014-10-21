#!/usr/bin/env python

from py2neo import neo4j, node, rel

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
            p = predicates.get_or_create("predicate",parent_name+" "+predicate_name+" "+child_name, (parent[0], predicate_name, child[0], data))
            return True
        else:
            return False

    def deletePredicateInstance(self, parent_name, child_name, predicate_name):
        entities = self.db.get_or_create_index(neo4j.Node, "Entities")
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")
        predicates = self.db.get_or_create_index(neo4j.Relationship, "Predicates")

        parent = entities.get("name",parent_name)
        child = entities.get("name",child_name)

        if not (len(parent) == 0) and not (len(child) == 0):
            print ("predicate",parent_name+" "+predicate_name+" "+child_name)
            p = predicates.get("predicate",parent_name+" "+predicate_name+" "+child_name)
            print p
            if len(p) > 0:
                self.db.delete(p[0])
                return True
            else:
                return False
        else:
            return False

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

        p = predicates.get_or_create("predicate",name + " CLASS", (entity, "IS-A", obj_class))

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


