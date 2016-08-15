#ifndef COMMAND_ADD_NODE_HPP
#define COMMAND_ADD_NODE_HPP

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT AddNode : public Command
{
public:
    AddNode(const AUUID &graph_uuid, const std::string& type, Point pos, const UUID& uuid_, NodeStatePtr state);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

private:
    std::string type_;
    Point pos_;

    UUID uuid_;

    NodeStatePtr saved_state_;
};
}
}

#endif // COMMAND_ADD_NODE_HPP
