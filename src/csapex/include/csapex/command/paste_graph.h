#ifndef PASTE_GRAPH_H
#define PASTE_GRAPH_H

/// COMPONENT
#include "command.h"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{


namespace command
{

class PasteGraph : public Command
{
public:
    PasteGraph(const YAML::Node& blueprint_, const Point &pos_);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    YAML::Node blueprint_;
    Point pos_;

    std::vector<UUID> inserted_;
};
}
}

#endif // PASTE_GRAPH_H
