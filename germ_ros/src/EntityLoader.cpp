#include <germ_ros/EntityLoader.h>

#include <iostream>
#include <map>

#include <XmlRpcException.h>

namespace germ_ros {

  std::string xmlGetString(XmlRpc::XmlRpcValue &xml, std::string id) {
    return static_cast<std::string>(xml[id]);
  }

  /**
   * Helper libary function to load entity specifications into C++ programs.
   * This is for programs that need world information to provide advanced functionality.
   * Uses XMLRPC.
   */
  std::vector<Entity> loadEntities(const std::string &path, const std::string &defset) {
    std::vector<Entity> entities;

    try{
      XmlRpc::XmlRpcValue defs; // set of all definitions

      ros::NodeHandle nh(path);
      nh.param("definitions", defs, defs);

      if (defs.hasMember(defset)) {

        XmlRpc::XmlRpcValue ds = defs[defset];

        if (ds.hasMember("entities")) {
          XmlRpc::XmlRpcValue defents = ds["entities"];

          for (unsigned int j = 0; j < defents.size(); ++j) {
            XmlRpc::XmlRpcValue ent = defents[j]; // the entity's s

            Entity e;
            e.name = xmlGetString(ent, "name");
            e.obj_class = xmlGetString(ent, "class");

            if (ent.hasMember("properties")) {
              for (unsigned int i = 0; i < ent["properties"].size(); ++i) {
                XmlRpc::XmlRpcValue temp = ent["properties"][i];
                e.data[xmlGetString(temp, "key")] = xmlGetString(temp, "value");
              }
            }

            ROS_INFO("Loaded entity with name=%s, class=%s, %u properties", e.name.c_str(), e.obj_class.c_str(), (unsigned int)e.data.size());
            entities.push_back(e);
          }
        }
      }

    } catch (XmlRpc::XmlRpcException const ex) {
      std::cout << ex.getMessage() << std::endl;
    }

    std::cout << "returning" << std::endl;
    return entities;
  }

  /**
   * Loads multiple sets of entities at once.
   */
  std::vector<Entity> loadEntities(const std::string &path, const std::vector<std::string> &defsets) {
    std::vector<Entity> entities;

    for (unsigned int i = 0; i < defsets.size(); ++i) {
      std::vector<Entity> etemp = loadEntities(path, defsets[i]);
      entities.insert(entities.begin(), etemp.begin(), etemp.end());
    }

    return entities;
  }

  void printEntity(const Entity &entity) {
      std::cout << "Name: " << entity.name << std::endl;
      std::cout << "Class: " << entity.obj_class << std::endl;
      std::cout << "Properties:" << std::endl;
      for (typename std::map<std::string, std::string>::const_iterator it = entity.data.begin();
           it != entity.data.end();
           ++it)
      {
        std::cout << "\t" << it->first << ": " << it->second << std::endl;
      }
  }

}
