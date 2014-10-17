#!/usr/bin/env python

from py2neo import neo4j, node, rel

import germ_msgs.msg as gm
import std_msgs.msg as sm

import rospy

class GraphInterface:

    '''
    add_predicate_instance()
    Process a single instantiated predicate into the system.
    '''
    def add_predicate_instance(self, msg):
        entities = self.db.get_or_create_index(neo4j.Node, "Entities")
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")
        predicates = self.db.get_or_create_index(neo4j.Relationship, "Predicates")

        parent = entities.get("name",msg.parent.name)
        child = entities.get("name",msg.child.name)

        if not (len(parent) == 0) and not (len(child) == 0):
            predicates.get_or_create("predicate",msg.parent.name+" "+msg.predicate.name+" "+msg.child.name, (parent[0], msg.predicate.name, child[0]))

    '''
    add_obj_cb()
    Adds an object with a class.
    '''
    def add_obj_cb(self, msg):

        entities = self.db.get_or_create_index(neo4j.Node, "Entities")
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")
        predicates = self.db.get_or_create_index(neo4j.Relationship, "ClassPredicates")

        entity = entities.get_or_create("name",msg.name, {"name":msg.name})
        entity.add_labels("entity")
        
        obj_class = classes.get_or_create("name", msg.obj_class, {"name":msg.obj_class})
        obj_class.add_labels("class")

        predicates.get_or_create("predicate",msg.name, (entity, "IS-A", obj_class))

    '''
    add_class_cb()
    Callback to add a single class to the database, as a string,
    '''
    def add_class_cb(msg):
        classes = self.db.get_or_create_index(neo4j.Node, "Classes")

        obj_class = classes.get_or_create("name", msg.data)
        obj_class.add_labels("class")

    '''
    add_predicate_cb()
    Class to force addition of a single instantiated predicate.
    This ignores the OPERATION field for now.
    '''
    def add_predicate_cb(self, msg):
        self.add_predicate_instance(msg)

    '''
    update_predicates_cb()
    Takes a whole list of predicates and saves them in the graph database.
    Uses the operation field to determine whether to add or delete.
    '''
    def update_predicates_cb(self, msg):
        for pred in msg.predicates:
            if pred.operation == gm.PredicateInstance.ADD:
                add_predicate_instance(pred)
            else:
                # find and remove this predicate; it's no longer valid
                pass

    def __init__(self, db_address="http://localhost:7474/db/data"):
        rospy.Subscriber("add_predicate", gm.PredicateInstance, self.add_predicate_cb)
        rospy.Subscriber("add_class", sm.String, self.add_class_cb)
        rospy.Subscriber("add_object", gm.Object, self.add_obj_cb)
        rospy.Subscriber("update_predicates", gm.PredicateInstanceList, self.update_predicates_cb)
        self.db = neo4j.GraphDatabaseService(db_address)

if __name__ == "__main__":
    rospy.init_node("germ_neo4j_interface")

    address = rospy.param("~db_address","http://localhost:7474/db/data")
    purge = rospy.param("~purge","false")

    rate = rospy.Rate(30)

    try:

        gi = GraphInterface(address)

        if purge == "true":
            gi.purge()
        elif not purge == "false":
            rospy.logwarn("Unknown value for argument \"purge\":"+purge)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
