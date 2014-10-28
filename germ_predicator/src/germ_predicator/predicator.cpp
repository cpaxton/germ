#include <germ_predicator/predicator.h>
#include <germ_ros/PredicateTools.h>

using namespace germ_ros;

#define CONCAT(x,y,z) (x + y + z)
#define KEY(p) (CONCAT(p.predicate.name, p.parent.name, p.child.name))

/**
  germ_predicator
  Can generate the set of predicates that would be true for a given location.
  Also used to generate predicates based on current state.
 **/
namespace germ_predicator {

  /*
   * joint_state_callback()
   * Update the robot state variable values
   */
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state) {
    state->setVariableValues(*msg);
  }

  PredicateContext::PredicateContext(bool publish) {
    ros::NodeHandle nh_tilde("~");
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue frames_list;
    XmlRpc::XmlRpcValue descriptions;
    XmlRpc::XmlRpcValue topics;
    XmlRpc::XmlRpcValue floating; // set of floating root joints that need to be updated

    nh_tilde.param("verbosity", verbosity, 1);
    nh_tilde.param("padding", padding, 0.01);
    nh_tilde.param("world_frame", world_frame, std::string("/world"));

    // should we publish predicate messages?
    // or what?
    if(publish == true) {
      pub = nh.advertise<germ_msgs::PredicateInstanceList>("update_predicates", 1000);
    }

    nh_tilde.param("rel_x_threshold", rel_x_threshold, 0.1);
    nh_tilde.param("rel_y_threshold", rel_y_threshold, 0.1);
    nh_tilde.param("rel_z_threshold", rel_z_threshold, 0.1);
    nh_tilde.param("near_2D_threshold", near_2d_threshold, 0.2);
    nh_tilde.param("near_3D_threshold", near_3d_threshold, 0.2);

    std::vector<std::string> category_list;
    category_list.push_back("robots");
    category_list.push_back("task");

    std::vector<germ_ros::Entity> entities = germ_ros::loadEntities("",category_list);

    // parse through specified entities
    for (unsigned int i = 0; i < entities.size(); ++i) {

      printEntity(entities[i]);

      if(entities[i].data.find("description") != entities[i].data.end()) {
        std::string desc = entities[i].data["description"];
        robot_names.push_back(entities[i].name);

        if(verbosity > 0) {
          ROS_INFO("Entity %d robot description parameter: %s", i, desc.c_str());
        }

        try {
          // create a robot model with state desc
          robot_model_loader::RobotModelLoader robot_model_loader(desc);
          ROS_INFO("Loaded model!");

          robot_model::RobotModelPtr model = robot_model_loader.getModel();
          PlanningScene *scene = new PlanningScene(model);
          scene->getCollisionRobotNonConst()->setPadding(padding);
          scene->propogateRobotPadding();

          // store robot information
          robots.push_back(model);
          scenes.push_back(scene);
          RobotState *state = new RobotState(model);
          states.push_back(state);

          if(entities[i].data.find("joint_states_topic") != entities[i].data.end()) {
            std::string topic = entities[i].data["joint_states_topic"];

            if(verbosity > 0) {
              ROS_INFO("Entity %d joint states topic: %s", i, topic.c_str());
            }

            // create the subscriber
            subs.push_back(nh.subscribe<sensor_msgs::JointState>
                           (topic, 1000,
                            boost::bind(joint_state_callback, _1, state)));
          } else {
            ROS_WARN("No joint states topic corresponding to description %s!", desc.c_str());
          }

          // add floating root frames if necessary
          if(entities[i].data.find("floating_frame") != entities[i].data.end()) {
            std::string floating_frame = entities[i].data["floating_frame"];
            floating_frames[entities[i].name] = floating_frame;
          }

        } catch (std::exception ex) {
          std::cerr << ex.what() << std::endl;
        }

      } else {
        ROS_WARN("No robot description parameter defined for entity %d (name=%s, class=%s)!", i, entities[i].name.c_str(), entities[i].obj_class.c_str());
      }

      if(entities[i].data.find("tf") != entities[i].data.end()) {
        std::string frame = entities[i].data["tf"];
        frames.push_back(frame);
        frames_to_entity_names[frame] = entities[i].name;

        if (verbosity > 0) {
          ROS_INFO("Added TF frame %s for entity %s (class %s)", frame.c_str(), entities[i].name.c_str(), entities[i].obj_class.c_str());
        }
      } else {
        ROS_WARN("No TF frame specified for entity %s!", entities[i].name.c_str());
      }

    }

    for(typename std::map<std::string, std::string>::iterator it = frames_to_entity_names.begin();
        it != frames_to_entity_names.end();
        ++it)
    {
      std::cout << it->first << " --> " << it->second << std::endl;
    }

    // print out information on all the different joints
    unsigned int i = 0;
    for (typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {
      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
      // -----------------------------------------------------------
      if (verbosity > 0) {
        std::cout << std::endl;
        std::cout << "PRINTING STATE INFO:";
        std::cout << robot1->getRobotModel()->getName() << std::endl;
        std::cout << robots[i]->getRootJointName() << std::endl;
      }
      states[i]->update(true);
      if (verbosity > 0) {
        states[i]->printStateInfo(std::cout);
      }
      // -----------------------------------------------------------
    }

    initializePredicateList();
  }

  /**
   * cleanup()
   * Delete memory allocated for robot states and subscribers
   */
  void PredicateContext::cleanup() {
    for (typename std::vector<RobotState *>::iterator it = states.begin();
         it != states.end();
         ++it)
    {
      delete *it;
    }

    for (typename std::vector<PlanningScene *>::iterator it = scenes.begin();
         it != scenes.end();
         ++it)
    {
      delete *it;
    }
  }

  /**
   * initializePredicateList()
   * Creates a map containing the truth of all predicates
   * This is used to generate a set of deletion messages for missing predicates
   */
  void PredicateContext::initializePredicateList() {

    unsigned int idx = 0;
    predicate_found.clear();

    if(verbosity > 0) {
      ROS_INFO("Computing list of predicates!");
    }
    for (typename std::map<std::string, std::string>::iterator it = frames_to_entity_names.begin();
         it != frames_to_entity_names.end();
         ++it)
    {
      for (typename std::map<std::string, std::string>::iterator it2 = frames_to_entity_names.begin();
           it2 != frames_to_entity_names.end();
           ++it2)
      {

        if(it->second == it2->second) {
          continue;
        }

        PredicateInstance pi;
        for (unsigned int i = 0; i < num_collision_predicates; ++i) {
          createPredicateInstance(pi, collision_predicates[i], it->first, it2->second, true);
          predicate_found[pi] = true;
        }
        for (unsigned int i = 0; i < num_geometry_predicates; ++i) {
          createPredicateInstance(pi, geometry_predicates[i], it->first, it2->second, true);
          predicate_found[pi] = true;
        }
      }
    }
  }


  /**
   * updatRobotStates()
   * make sure base frames are up to date
   * some objects, such as free-floating robots (aka the ring) need to be updated by TF
   * not sure why this doesn't work naturally
   */
  void PredicateContext::updateRobotStates() {
    unsigned int i = 0;


    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
      std::string name = robot1->getRobotModel()->getName();

      if(floating_frames.find(name) != floating_frames.end()) {
        std::string base_frame = floating_frames[name];

        tf::StampedTransform transform;
        Eigen::Affine3d t;

        try{
          listener.lookupTransform(world_frame, base_frame,
                                   ros::Time(0), transform);
          tf::transformTFToEigen(transform,t);
          states[i]->setJointPositions(robot1->getRobotModel()->getRootJointName(), t);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        if(verbosity > 1) {
          std::cout << "----------------------------" << std::endl;
          std::cout << "PRINTING STATE INFO:";
          std::cout << robot1->getRobotModel()->getName() << std::endl;
          std::cout << robots[i]->getRootJointName() << std::endl;
          states[i]->update(true);
          states[i]->printStateInfo(std::cout);
        }

      } else {
        continue;
      }
    }
  }

  /**
   * addCollisionPredicates()
   * main collision checking loop
   * checks for all pairs of objects, determines collisions and distances
   * publishes the relationships between all of these objects
   */
  void PredicateContext::addCollisionPredicates(germ_msgs::PredicateInstanceList &output, const std::vector<RobotState *> &states, unsigned int idx) {

    unsigned i = 0;
    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {

      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();

      typename std::vector<PlanningScene *>::iterator it2 = it1;
      unsigned int j = i+1;

      // skip if we are only computing predicates for a single planning scene
      if (idx < scenes.size() && i != idx) {
        continue;
      } else if (idx < scenes.size() && i == idx){
        j = 0;
        it2 = scenes.begin();
      }

      for(++it2; it2 != scenes.end(); ++it2, ++j) {

        if (i == j) continue;

        collision_detection::CollisionRobotConstPtr robot2 = (*it2)->getCollisionRobot();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.contacts = true;
        req.max_contacts = 1000;

        // force an update
        // source: https://groups.google.com/forum/#!topic/moveit-users/O9CEef6sxbE
        states[i]->update(true);
        states[j]->update(true);

        robot1->checkOtherCollision(req, res, *states[i], *robot2, *states[j]);
        double dist = robot1->distanceOther(*states[i], *robot2, *states[j]);

        /*
        PredicateStatement ps = createStatement("touching", -1.0 * dist,
                                                robot1->getRobotModel()->getName(),
                                                robot2->getRobotModel()->getName());
        PredicateStatement ps2 = createStatement("touching", -1.0 * dist,
                                                 robot2->getRobotModel()->getName(),
                                                 robot1->getRobotModel()->getName());
        */

        if (verbosity > 4) {
          std::cout << res.contacts.size() << " contacts found" << std::endl;
        }

        // iterate over all collisions
        for(collision_detection::CollisionResult::ContactMap::const_iterator cit = res.contacts.begin(); 
            cit != res.contacts.end(); 
            ++cit)
        {
          
          if(frames_to_entity_names.find(cit->first.second) != frames_to_entity_names.end()
             && frames_to_entity_names.find(cit->first.first) != frames_to_entity_names.end())
          {
            germ_msgs::PredicateInstance pi;
            createPredicateInstance(pi, collision_predicates[0],
                                    frames_to_entity_names[cit->first.second],
                                    frames_to_entity_names[cit->first.first],
                                    true, true);
            if(verbosity > 0) {
              ROS_printPredicate(pi);
            }
            output.predicates.push_back(pi);
          }
        }

        if (verbosity > 1) {
          std::cout << "(" << robot1->getRobotModel()->getName()
            << ", " << robot2->getRobotModel()->getName()
            << ") : Distance to collision: " << dist << std::endl;
        }
      }
    }

  }

  /**
   * tick()
   * Run one iteration of the predicator computations 
   */
  void PredicateContext::tick() {
    germ_msgs::PredicateInstanceList output;
    output.info.name = ros::this_node::getName();

    updateRobotStates();
    addCollisionPredicates(output, states);
    addGeometryPredicates(output, states);
    addReachabilityPredicates(output, states);

    pub.publish(output);
  }

  /**
   * addReachabilityPredicates()
   * compute whether or not we can reach certain points or waypoints
   */
  void PredicateContext::addReachabilityPredicates(germ_msgs::PredicateInstanceList &output, const std::vector<RobotState *> &states) {
    // update list of reachable waypoints
    // use a service call to predicator to get the relevant waypoints

    // compute whether or not that point can be reached
  }

  /**
   * getLinkTransform
   * Check to see if this is in the list of floating transfoms
   * If so, compose with TF frame
   * NOTE: actually, it looks like we don't need this at all
   */
  Eigen::Affine3d PredicateContext::getLinkTransform(const RobotState *state, const std::string &linkName) const {

    std::string name = state->getRobotModel()->getName();
    Eigen::Affine3d tf1 = state->getGlobalLinkTransform(linkName);

    return tf1;
  }

  /**
   * addGeometryPredicates()
   * compute the set of geometry predicates
   *
   * Links for the different world objects from the peg demo:
   world wam/base_link wam/shoulder_yaw_link wam/shoulder_pitch_link wam/upper_arm_link wam/forearm_link wam/wrist_yaw_link wam/wrist_pitch_link wam/wrist_palm_link wam/hand/bhand_palm_link wam/hand/bhand_grasp_link wam/hand/bhand_palm_surface_link wam/hand/finger_1/prox_link wam/hand/finger_1/med_link wam/hand/finger_1/dist_link wam/hand/finger_2/prox_link wam/hand/finger_2/med_link wam/hand/finger_2/dist_link wam/hand/finger_3/med_link wam/hand/finger_3/dist_link wam/wrist_palm_stump_link 
   world wam2/base_link wam2/shoulder_yaw_link wam2/shoulder_pitch_link wam2/upper_arm_link wam2/forearm_link wam2/wrist_yaw_link wam2/wrist_pitch_link wam2/wrist_palm_link wam2/hand/bhand_palm_link wam2/hand/bhand_grasp_link wam2/hand/bhand_palm_surface_link wam2/hand/finger_1/prox_link wam2/hand/finger_1/med_link wam2/hand/finger_1/dist_link wam2/hand/finger_2/prox_link wam2/hand/finger_2/med_link wam2/hand/finger_2/dist_link wam2/hand/finger_3/med_link wam2/hand/finger_3/dist_link wam2/wrist_palm_stump_link 
   world peg1/base_link peg1/peg_link peg1/peg_top_link 
   world peg2/base_link peg2/peg_link peg2/peg_top_link 
   ring1/ring_link 
   world stage_link 
   */
  void PredicateContext::addGeometryPredicates(germ_msgs::PredicateInstanceList &output, const std::vector<RobotState *> &states) {

    unsigned int i = 0;
    for(typename std::vector<RobotState *>::const_iterator it = states.begin();
        it != states.end();
        ++it, ++i)
    {

      // get the list of joints for the robot state
      for (typename std::vector<std::string>::const_iterator link1 = (*it)->getRobotModel()->getLinkModelNames().begin();
           link1 != (*it)->getRobotModel()->getLinkModelNames().end();
           ++link1)
      {
        if (link1->compare(std::string("world")) == 0) {
          continue;
        } else if (frames_to_entity_names.find(*link1) == frames_to_entity_names.end()) {
          continue;
        }
        std::string link1Entity = frames_to_entity_names[*link1];

        // access world coordinates
        // NOTE: does not work for the ring yet!
        Eigen::Affine3d tf1 = getLinkTransform(*it, *link1);

        // loop over the other objects in the world
        // this does NOT include waypoints or anything like that -- we need a separate loop
        // the second loop can handle abstract entities like these
        unsigned int j = 0;
        for(typename std::vector<RobotState *>::const_iterator it2 = states.begin();
            it2 != states.end();
            ++it2, ++j)
        {
          if (i == j) {
            continue;
          }

          // loop over the non-world links of this object
          // get the list of joints for the robot state
          for (typename std::vector<std::string>::const_iterator link2 = (*it2)->getRobotModel()->getLinkModelNames().begin();
               link2 != (*it2)->getRobotModel()->getLinkModelNames().end();
               ++link2)
          {
            if (link2->compare(std::string("world")) == 0) {
              continue;
            }else if (frames_to_entity_names.find(*link2) == frames_to_entity_names.end()) {
              continue;
            }
            std::string link2Entity = frames_to_entity_names[*link2];

            Eigen::Affine3d tf2 = getLinkTransform(*it2, *link2);

            if (verbosity > 2) {
              std::cout << *link1 << ", " << *link2 << std::endl;
            }

            if (verbosity > 3) {
              std::cout << tf1.translation()[0] << "," << tf1.translation()[1] << "," << tf1.translation()[2] << " --> ";
              std::cout << tf2.translation()[0] << "," << tf2.translation()[1] << "," << tf2.translation()[2] << std::endl;
            }

            double xdiff = tf1.translation()[1] - tf2.translation()[1]; // x = red = front/back from stage
            double ydiff = tf1.translation()[0] - tf2.translation()[0]; // y = green = left/right?
            double zdiff = tf1.translation()[2] - tf2.translation()[2]; // z = blue = up/down
            double dist_xy = sqrt((xdiff*xdiff) + (ydiff*ydiff)); // compute xy distance only
            double dist = sqrt((xdiff*xdiff) + (ydiff*ydiff) + (zdiff*zdiff)); // compute xyz distance

            PredicateInstance wleft, wright, wfront, wback, wup, wdown, near, near_xy;

            // x is left/right
            if (xdiff < -1.0 * rel_x_threshold){
              createPredicateInstance(wright,
                                      geometry_predicates[WORLD_REF_IDX + RIGHT_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              output.predicates.push_back(wright);
            } else if (xdiff > rel_x_threshold) {
              createPredicateInstance(wleft,
                                      geometry_predicates[WORLD_REF_IDX + LEFT_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              output.predicates.push_back(wleft);
            }

            // y is front/back
            if (ydiff < -1.0 * rel_y_threshold) {
              createPredicateInstance(wback,
                                      geometry_predicates[WORLD_REF_IDX + BACK_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              output.predicates.push_back(wback);
            } else if (ydiff > rel_y_threshold) {
              createPredicateInstance(wfront,
                                      geometry_predicates[WORLD_REF_IDX + FRONT_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              output.predicates.push_back(wfront);
            }

            // z is front/back
            if (zdiff < -1.0 * rel_z_threshold) {
              createPredicateInstance(wdown,
                                      geometry_predicates[WORLD_REF_IDX + BELOW_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              output.predicates.push_back(wdown);
            } else if (zdiff > rel_z_threshold) {
              createPredicateInstance(wup,
                                      geometry_predicates[WORLD_REF_IDX + ABOVE_OFFSET],
                                      link1Entity,
                                      link2Entity,
                                      true, false);
              ROS_printPredicate(wup);
              output.predicates.push_back(wup);
            }

            if (dist < near_3d_threshold) {
              createPredicateInstance(near,
                                      geometry_predicates[NEAR_IDX],
                                      link1Entity,
                                      link2Entity,
                                      true, true);
              output.predicates.push_back(near);
            }

            if (dist_xy < near_2d_threshold) {
              createPredicateInstance(near_xy,
                                      geometry_predicates[NEAR_XY_IDX],
                                      link1Entity,
                                      link2Entity,
                                      true, true);
              output.predicates.push_back(near_xy);
            }
          }
        }
      }
    }
  }
}
