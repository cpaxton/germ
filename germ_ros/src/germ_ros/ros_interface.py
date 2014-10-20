#!/usr/bin/env python


import rospy
import germ_msgs as gm
import std_msgs as sm

from germ_neo4j import GermDatabaseConnection

class GermROSListener:
    
    def __init__(self):
        dbc = GermDatabaseConnection()


