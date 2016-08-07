#ifndef CSAPEX_YAML_IO_HPP
#define CSAPEX_YAML_IO_HPP

/// PROJECT
#include <csapex/msg/token_traits.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <memory>
#include <yaml-cpp/yaml.h>

namespace YAML {

template<typename T, typename Enable = void>
struct convertPtr
{
    static Node encode(const std::shared_ptr<T> rhs)
    {
        return Node (*rhs);
    }

    static bool decode(const Node& node, std::shared_ptr<T>& rhs)
    {
        rhs = csapex::connection_types::makeEmpty<T>();
        return YAML::convert<T>::decode(node, *rhs);
    }
};

template<typename T, typename Enable = void>
struct convertConstPtr
{
    static Node encode(const std::shared_ptr<T const> rhs)
    {
        return Node (*rhs);
    }

    static bool decode(const Node& node, std::shared_ptr<T const>& rhs)
    {
        auto tmp = csapex::connection_types::makeEmpty<T>();
        auto res = YAML::convert<T>::decode(node, *tmp);
        rhs = tmp;
        return res;
    }

};

template<typename T>
struct convert< std::shared_ptr<T> > {
  static Node encode(const std::shared_ptr<T> rhs)
  {
      return YAML::convertPtr<T>::encode(rhs);
  }

  static bool decode(const Node& node, std::shared_ptr<T>& rhs)
  {
      return YAML::convertPtr<T>::decode(node, rhs);
  }
};

template<typename T>
struct convert< std::shared_ptr<T const> > {
  static Node encode(const std::shared_ptr<T const> rhs)
  {
      return YAML::convertConstPtr<T>::encode(rhs);
  }

  static bool decode(const Node& node, std::shared_ptr<T const>& rhs)
  {
      return YAML::convertConstPtr<T>::decode(node, rhs);
  }
};

//template<>
//struct convert<csapex::UUID> {
//    static Node encode(const csapex::UUID& rhs) {
//        return Node (rhs.getFullName());
//    }

//    static bool decode(const Node& node, csapex::UUID& rhs) {
//        rhs = csapex::UUIDProvider::makeUUID_forced(std::weak_ptr<csapex::UUIDProvider>(), node.as<std::string>());
//        return true;
//    }
//};

}

#endif // CSAPEX_YAML_IO_HPP
