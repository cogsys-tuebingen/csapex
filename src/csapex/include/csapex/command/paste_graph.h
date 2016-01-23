#ifndef PASTE_GRAPH_H
#define PASTE_GRAPH_H

/// COMPONENT
#include "meta.h"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace csapex
{


namespace command
{

class PasteGraph : public Meta
{
public:
    PasteGraph(const UUID& graph_id, const YAML::Node& blueprint, const Point &pos);
    PasteGraph(const UUID& graph_id, const YAML::Node& blueprint, const Point &pos,
               const std::vector<std::pair<UUID,UUID>>& crossing_inputs,
               const std::vector<std::pair<UUID,UUID>>& crossing_outputs);

    std::unordered_map<UUID, UUID, UUID::Hasher> getMapping() const;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    YAML::Node blueprint_;
    Point pos_;

    std::unordered_map<UUID, UUID, UUID::Hasher> id_mapping_;

    std::vector<std::pair<UUID,UUID>> crossing_inputs_;
    std::vector<std::pair<UUID,UUID>> crossing_outputs_;
};
}
}

#endif // PASTE_GRAPH_H
