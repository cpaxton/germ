#ifndef GERM_ENTITY_LOADER
#define GERM_ENTITY_LOADER

#include <string>
#include <vector>

#include <ros/ros.h>
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

  /**
   * Helper libary function to load entity specifications into C++ programs.
   * This is for programs that need world information to provide advanced functionality.
   * Uses XMLRPC.
   */
  std::vector<Entity> loadEntities(const std::string &path = std::string(""), const std::string &defset = std::string("robots"));

  /**
   * Loads multiple sets of entities at once.
   */
  std::vector<Entity> loadEntities(const std::string &path, const std::vector<std::string> &defsets);

}
#endif
