#ifndef CSAPEX_YAML_IO_HPP
#define CSAPEX_YAML_IO_HPP

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
      rhs.reset(new T);
      return YAML::convert<T>::decode(node, *rhs);
  }
};
}

#endif // CSAPEX_YAML_IO_HPP
