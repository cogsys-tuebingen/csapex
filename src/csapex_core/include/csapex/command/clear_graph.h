#ifndef CLEAR_GRAPH_H
#define CLEAR_GRAPH_H

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT ClearGraph : public Meta
{
    COMMAND_HEADER(ClearGraph);

public:
    ClearGraph(const AUUID& graph_uuid);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(ClearGraph));
    }

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    std::string getDescription() const override;
};

}  // namespace command
}  // namespace csapex

#endif  // CLEAR_GRAPH_H
