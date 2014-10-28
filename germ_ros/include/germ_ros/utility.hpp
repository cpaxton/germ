#ifndef _PP_UTILITY
#define _PP_UTILITY

#include <unordered_map>
#include <germ_msgs/PredicateInstance.h>
#include <string>
#include <functional>

using std::string;
using std::unordered_map;
using namespace germ_msgs;

namespace germ_ros {

  struct Hash {

    std::hash<std::string> hash_str;

    Hash() : hash_str() {}

    size_t operator()(const PredicateInstance &msg) const {

      size_t res = hash_str(msg.predicate.name);
      res += hash_str(msg.parent.name) << 3;
      res += hash_str(msg.child.name) << 6;

      return res;
    }
  };

  struct Equals {
    bool operator()(const PredicateInstance &msg1,
                    const PredicateInstance &msg2) const
    {
      return msg1.predicate.name == msg2.predicate.name &&
        msg1.parent.name == msg2.parent.name &&
        msg1.child.name == msg2.child.name;
    }
  };

  struct HashOperation {

    std::hash<std::string> hash_str;

    HashOperation() : hash_str() {}

    size_t operator()(const PredicateInstance &msg) const {

      size_t res = hash_str(msg.predicate.name);
      res += hash_str(msg.parent.name) << 3;
      res += hash_str(msg.child.name) << 6;
      res ^= (int)(msg.operation == PredicateInstance::REMOVE) << 10;

      return res;
    }
  };

  struct EqualsOperation {
    bool operator()(const PredicateInstance &msg1,
                    const PredicateInstance &msg2) const
    {
      return msg1.predicate.name == msg2.predicate.name &&
        msg1.parent.name == msg2.parent.name &&
        msg1.child.name == msg2.child.name &&
        msg1.operation == msg2.operation;
    }
  };
}

#endif
