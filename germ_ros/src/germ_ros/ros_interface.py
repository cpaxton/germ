#!/usr/bin/env python

import rospy
import germ_msgs.msg as gm
import std_msgs.msg as sm

from germ_neo4j import GermDatabaseConnection

def get_properties(props, name=""):
    data = {}
    for i in range(len(props.key)):
        k = props.key[i]
        v = props.value[i]
        t = props.type[i]

        if len(v) == 0:
            continue

        if t == "float":
            data[k] = float(v)
        else:
            if not t == "string":
                rospy.logwarn("Unrecognized type: %s"%(t))
            data[k] = v

    if not len(name) == 0:
        data["name"] = name

class GermROSListener:
    
    def __init__(self, db_address="http://localhost:7474/db/data"):
        self.dbc = GermDatabaseConnection(address)
        rospy.Subscriber("add_predicate", gm.PredicateInstance, self.add_predicate_cb)
        rospy.Subscriber("add_class", sm.String, self.add_class_cb)
        rospy.Subscriber("add_object", gm.Object, self.add_obj_cb)
        rospy.Subscriber("update_predicates", gm.PredicateInstanceList, self.update_predicates_cb)

    '''
    add_obj_cb()
    Adds an object with a class.
    '''
    def add_obj_cb(self, msg):
        data = get_properties(msg.data, msg.name)
        self.dbc.addObject(msg.name, msg.obj_class, data)

    '''
    add_class_cb()
    Callback to add a single class to the database, as a string,
    '''
    def add_class_cb(self, msg):
        self.dbc.addClass(msg.data)

    '''
    add_predicate_cb()
    Class to force addition of a single instantiated predicate.
    This ignores the OPERATION field for now.
    '''
    def add_predicate_cb(self, msg):
        data = get_properties(msg.data, msg.predicate.name)
        self.dbc.addPredicateInstance(msg.parent.name, msg.child.name, msg.predicate.name, data)

    '''
    update_predicates_cb()
    Takes a whole list of predicates and saves them in the graph database.
    Uses the operation field to determine whether to add or delete.
    '''
    def update_predicates_cb(self, msg):
        for pred in msg.predicates:
            if pred.operation == gm.PredicateInstance.ADD:
                data = get_properties(pred.data, pred.predicate.name)
                addPredicateInstance(pred.parent.name, pred.child.name, pred.predicate.name, data)
            else:
                # find and remove this predicate; it's no longer valid
                rospy.logerr("Remove not yet implemented!")
                pass



if __name__ == "__main__":
    rospy.init_node("germ_ros_interface")

    address = rospy.get_param("~db_address","http://localhost:7474/db/data")
    purge = rospy.get_param("~purge","false")

    rate = rospy.Rate(30)

    try:

        gi = GermROSListener(address)

        if purge == "true":
            rospy.logwarn("Deleting current database! Hope you backed up anything important!")
            gi.dbc.purge()
        elif not purge == "false":
            rospy.logwarn("Unknown value for argument \"purge\":"+purge)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

