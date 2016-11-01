#ifndef PASTE_GRAPH_H
#define PASTE_GRAPH_H

/// COMPONENT
#include "meta.h"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{


namespace command
{

class CSAPEX_COMMAND_EXPORT PasteGraph : public Meta
{
public:
    PasteGraph(const AUUID &graph_id, const Snippet& blueprint, const Point &pos);

    std::unordered_map<UUID, UUID, UUID::Hasher> getMapping() const;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    Snippet blueprint_;
    Point pos_;

    std::unordered_map<UUID, UUID, UUID::Hasher> id_mapping_;
};
}
}

#endif // PASTE_GRAPH_H
