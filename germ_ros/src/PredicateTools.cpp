#include <germ_ros/PredicateTools.h>

using namespace germ_msgs;

namespace germ_ros {

  void ROS_printPredicate(const germ_msgs::PredicateInstance &pi) {
    if (!pi.is_bidirectional) {
      ROS_INFO("PredicateInstance: %s ---[ %s ]---> %s", pi.parent.name.c_str(), pi.predicate.name.c_str(), pi.child.name.c_str());
    } else {
      ROS_INFO("PredicateInstance: %s <---[ %s ]---> %s", pi.parent.name.c_str(), pi.predicate.name.c_str(), pi.child.name.c_str());
    }
  }

  void createPredicateInstance(PredicateInstance &pi,
                               const std::string &predicate_name,
                               const std::string &parent_name,
                               const std::string &child_name,
                               const bool add,
                               const bool isBidirectional)
  {
    pi.predicate.name = std::string(predicate_name);
    pi.parent.name = std::string(parent_name);
    pi.child.name = std::string(child_name);
    pi.operation = PredicateInstance::ADD;
    pi.is_bidirectional = isBidirectional;
  }

}
