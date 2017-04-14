#ifndef PASTE_GRAPH_H
#define PASTE_GRAPH_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{


namespace command
{

class CSAPEX_COMMAND_EXPORT PasteGraph : public CommandImplementation<PasteGraph>
{
    COMMAND_HEADER(PasteGraph);

public:
    PasteGraph(const AUUID &graph_id, const Snippet& blueprint, const Point &pos);

    std::unordered_map<UUID, UUID, UUID::Hasher> getMapping() const;

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(PasteGraph));
    }

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

protected:
    SnippetPtr blueprint_;
    Point pos_;

    CommandPtr delete_command_;

    std::unordered_map<UUID, UUID, UUID::Hasher> id_mapping_;
};
}
}

#endif // PASTE_GRAPH_H
