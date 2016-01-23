#ifndef COMMAND_DELETE_NODE_H
#define COMMAND_DELETE_NODE_H

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{
class DeleteNode : public Meta
{
public:
    DeleteNode(const UUID &parent_uuid, const UUID &uuid);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    std::string type;
    UUID uuid;

//    Command::Ptr remove_connections;

    NodeStatePtr saved_state;
};

}
}
#endif // COMMAND_DELETE_NODE_H
