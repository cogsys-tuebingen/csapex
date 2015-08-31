#ifndef SWITCH_THREAD_H
#define SWITCH_THREAD_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct SwitchThread : public Command
{
    SwitchThread(const UUID& node, int thread_id);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    UUID uuid;
    int old_id;
    int id;
    std::string name;
};

}

}
#endif // SWITCH_THREAD_H

