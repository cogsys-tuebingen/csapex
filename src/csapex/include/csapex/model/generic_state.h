#ifndef GENERIC_STATE_H
#define GENERIC_STATE_H

/// COMPONENT
#include <csapex/model/memento.h>

/// PROJECT
#include <utils_param/parameter_map.h>

namespace csapex
{

struct GenericState : public Memento {
    typedef boost::shared_ptr<GenericState> Ptr;

    std::map<std::string, param::Parameter::Ptr> params;

    virtual void writeYaml(YAML::Emitter& out) const;
    virtual void readYaml(const YAML::Node& node);
};

}

#endif // GENERIC_STATE_H
