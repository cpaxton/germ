#ifndef GERM_ENTITY_LOADER
#define GERM_ENTITY_LOADER

#include <string>
#include <vector>

#include <XmlRpcValue.h>

/**
 * Simple tool for loading entity data from the ROS parameter server.
 */
namespace germ_ros {

  /**
   * Struct defining an entity to track and manage
   */
  struct Entity {
    std::string obj_class;
    std::string name;
    std::map<std::string, std::string> data;
  };

  std::vector<Entity> loadEntities(std::string path = "");

}
#endif
