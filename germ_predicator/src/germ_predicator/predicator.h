#ifndef _PREDICATOR_PLANNING
#define _PREDICATOR_PLANNING

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <XmlRpcValue.h>

// for debugging
#include <iostream>

// stl
#include <vector>
#include <set>

// MoveIt!
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// joint states
#include <sensor_msgs/JointState.h>

// predicator
#include <germ_msgs/PredicateInstanceList.h>
#include <germ_msgs/PredicateInstance.h>
#include <germ_msgs/Predicate.h>
#include <germ_msgs/Object.h>

// boost includes
#include <boost/bind/bind.hpp>

#include "utility.hpp"

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;

namespace germ_predicator {

  /**
   * Predicate names
   */
  const std::string collision_predicates[] = {"TOUCHING", "NEAR-MESH"};
  const std::string geometry_predicates[] = {"NEAR", "NEAR-XY", "ABOVE", "BELOW", "LEFT-OF", "RIGHT-OF", "IN-FRONT-OF", "IN-BACK-OF",
    "WORLD-ABOVE", "WORLD-BELOW", "WORLD-LEFT-OF", "WORLD-RIGHT-OF", "WORLD-IN-FRONT-OF", "WORLD-IN-BACK-OF"};
  const std::string reachable_predicates[] = {"IS-REACHABLE"};

  /**
   * Indices for finding names of predicates
   */
  const unsigned int NEAR_IDX = 0;
  const unsigned int NEAR_XY_IDX = 1;
  const unsigned int ABOVE_OFFSET = 0;
  const unsigned int BELOW_OFFSET = 1;
  const unsigned int LEFT_OFFSET = 2;
  const unsigned int RIGHT_OFFSET = 3;
  const unsigned int FRONT_OFFSET = 4;
  const unsigned int BACK_OFFSET = 5;
  const unsigned int LOCAL_REF_IDX = 2;
  const unsigned int WORLD_REF_IDX = 8;


  typedef std::unordered_map<germ_msgs::PredicateInstance,
          unsigned int,
          germ_predicator::Hash,
          germ_predicator::Equals> heuristic_map_t;

  /*
   * joint_state_callback()
   * Update the robot state variable values
   */
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state);

  /**
   * ValueItem
   * A value-type pair stored as extra information about an object or predicate
   */
  struct ValueItem {
    std::string type;
    std::string value;
  }

  /**
   * Predicate
   * Simple internal representation of a predicate
   */
  struct Predicate : std::map<std::string, ValueItem> {
    std::string predicate;
    std::string parent;
    std::string child;

    Predicate(std::string predicate, std::string parent, std::string child);
  };

  /**
   * PredicateContext
   *
   */
  struct PredicateContext {
    std::vector<std::string> frames; // frames that we will look at for geometry predicates
    std::vector<RobotModelPtr> robots; // robot models
    std::vector<RobotState *> states; // configured robot states
    std::vector<PlanningScene *> scenes;
    std::vector<ros::Subscriber> subs;

    double rel_x_threshold;
    double rel_y_threshold;
    double rel_z_threshold;
    double near_2d_threshold;
    double near_3d_threshold;

    tf::TransformListener listener;

    double padding; // how much padding do we give robot links?
    int verbosity; // how much should get printed for debugging purposes

    /*
     * heuristic_indices
     * Stores the locations in a double array of the indices for different features (heuristics)
     */
    heuristic_map_t heuristic_indices;

    std::map<std::string, std::string> floating_frames;
    std::string world_frame;


    ros::Publisher pub;

    /**
     * Create a PredicateContext()
     * Sets up space, collision robots, etc.
     * This will produce the low-level world predicates
     */
    PredicateContext(bool publish);

    /**
     * numHeuristics()
     */
    size_t numHeuristics() const;

    /**
     * updateWaypoints()
     * Get the list of waypoints from predicator
     * These are for reachability I guess
     */
    void updateWaypoints();

    /**
     * cleanup()
     * Delete memory allocated for robot states and subscribers
     */
    void cleanup();

    /**
     * tick()
     * Run one iteration of the predicator computations 
     */
    void tick();

    /**
     * updatRobotStates()
     * make sure base frames are up to date
     * some objects, such as free-floating robots (aka the ring) need to be updated by TF
     * not sure why this doesn't work naturally
     */
    void updateRobotStates();

    /**
     * addCollisionPredicates()
     * main collision checking loop
     * checks for all pairs of objects, determines collisions and distances
     * publishes the relationships between all of these objects
     *
     * @param idx is the index of a particular PlanningScene.
     * When doing planning, we don't really need to recompute all of the world collisions, just the ones that might be changing.
     */
    void addCollisionPredicates(const std::vector<RobotState *> &states, unsigned int idx=~0);

    /**
     * addGeometryPredicates()
     * compute the set of geometry predicates
     */
    void addGeometryPredicates(const std::vector<RobotState *> &states);

    /**
     * addReachabilityPredicates()
     * compute whether or not we can reach certain points or waypoints
     */
    void addReachabilityPredicates(const std::vector<RobotState *> &states);

    /**
     * getLinkTransform
     * Check to see if this is in the list of floating transfoms
     * If so, compose with TF frame
     */
    Eigen::Affine3d getLinkTransform(const RobotState *state, const std::string &linkName) const;


    /**
     * updateIndices()
     * Records where the values we can use as heuristics are going to be stored.
     * May also look at things like waypoints, etc.
     */
    void updateIndices();

    /**
     * getHeuristic
     * Looks up a score from a vector of possible values
     */
    //double getHeuristic(const PredicateStatement &pred, const std::vector<double> &heuristics) const;
  };
}

#endif
