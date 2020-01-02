#ifndef SET_COLOR_H
#define SET_COLOR_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT SetColor : public CommandImplementation<SetColor>
{
    COMMAND_HEADER(SetColor);

public:
    SetColor(const AUUID& graph_uuid, const UUID& node, int r, int g, int b);

    std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;

    int r;
    int g;
    int b;

    int r_orig;
    int g_orig;
    int b_orig;
};

}  // namespace command

}  // namespace csapex
#endif  // SET_COLOR_H
