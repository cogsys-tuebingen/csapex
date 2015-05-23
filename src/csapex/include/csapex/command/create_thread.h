#ifndef CREATE_THREAD_H
#define CREATE_THREAD_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CreateThread : public Command
{
    CreateThread(const UUID& node, const std::string &name);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    UUID uuid;
    std::string name;

    int old_id;
    int new_id;
};

}

}
#endif // CREATE_THREAD_H

