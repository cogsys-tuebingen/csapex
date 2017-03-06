#ifndef MODIFY_THREAD_H
#define MODIFY_THREAD_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT ModifyThread : public Command
{
    ModifyThread(int thread_id, const std::string& name);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int id;
    std::string name;
    std::string old_name;
};

}

}

#endif // MODIFY_THREAD_H
