#ifndef CSAPEX_YAML_IO_HPP
#define CSAPEX_YAML_IO_HPP

/// PROJECT
#include <csapex/msg/message_traits.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>

namespace YAML {
template<typename T>
struct convert< boost::shared_ptr<T> > {
  static Node encode(const boost::shared_ptr<T> rhs)
  {
      return Node (*rhs);
  }

  static bool decode(const Node& node, boost::shared_ptr<T>& rhs)
  {
      init(rhs);
      return YAML::convert<T>::decode(node, *rhs);
  }

  static void init(boost::shared_ptr<T>& rhs)
  {
      rhs = csapex::connection_types::makeEmpty<T>();
  }
//  static void init(boost::shared_ptr<T>& rhs)
//  {
//      rhs.reset(new T);
//  }
};

template<>
struct convert<csapex::UUID> {
    static Node encode(const csapex::UUID& rhs) {
        return Node (rhs.getFullName());
    }

    static bool decode(const Node& node, csapex::UUID& rhs) {
        rhs = csapex::UUID::make_forced(node.as<std::string>());
        return true;
    }
};

}

#endif // CSAPEX_YAML_IO_HPP
