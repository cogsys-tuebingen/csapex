#ifndef DELETE_THREAD_H
#define DELETE_THREAD_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT DeleteThread : public Command
{
    DeleteThread(int thread_id);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int id;
    std::string name;
};

}

}

#endif // DELETE_THREAD_H
