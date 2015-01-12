#ifndef MEMENTO_H
#define MEMENTO_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

/// FORWARD DECLARATION
namespace YAML
{
class Node;
}

namespace csapex
{

class Memento
{
public:
    typedef boost::shared_ptr<Memento> Ptr;

public:
    Memento();
    virtual ~Memento();

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);
};

}

#endif // MEMENTO_H
