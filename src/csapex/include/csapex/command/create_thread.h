#ifndef CREATE_THREAD_H
#define CREATE_THREAD_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT CreateThread : public Command
{
    CreateThread(const AUUID &graph_uuid, const UUID& node, const std::string &name);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    std::string name;

    int old_id;
    int new_id;
};

}

}
#endif // CREATE_THREAD_H

