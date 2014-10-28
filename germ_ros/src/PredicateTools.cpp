#include <germ_ros/PredicateTools.h>

using namespace germ_msgs;

namespace germ_ros {

  void ROS_printPredicate(const germ_msgs::PredicateInstance &pi) {
    ROS_INFO("PredicateInstance: %s ---[ %s ]---> %s", pi.parent.name.c_str(), pi.predicate.name.c_str(), pi.child.name.c_str());
  }

  PredicateInstance createPredicateInstance(const std::string &predicate_name,
                                            const std::string &parent_name,
                                            const std::string &child_name,
                                            const bool add)
  {
    PredicateInstance pi;

    pi.predicate.name = predicate_name;
    pi.parent.name = parent_name;
    pi.child.name = child_name;
    pi.operation = PredicateInstance::ADD;
  }

}
