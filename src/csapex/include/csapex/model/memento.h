#ifndef MEMENTO_H
#define MEMENTO_H

/// PROJECT
#include <csapex/csapex_export.h>

/// SYSTEM
#include <memory>

/// FORWARD DECLARATION
namespace YAML
{
class Node;
}

namespace csapex
{

class CSAPEX_EXPORT Memento
{
public:
    typedef std::shared_ptr<Memento> Ptr;

public:
    Memento();
    virtual ~Memento();

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);
};

}

#endif // MEMENTO_H
