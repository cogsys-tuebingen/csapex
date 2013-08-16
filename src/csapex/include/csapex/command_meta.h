#ifndef COMMAND_META_H
#define COMMAND_META_H

/// COMPONENT
#include "command.h"

/// SYSTEM
#include <vector>

namespace csapex
{
namespace command
{

struct Meta : public Command {
    typedef boost::shared_ptr<Meta> Ptr;

    Meta();
    void add(Command::Ptr cmd);

    int commands() const;

protected:
    bool execute();
    bool undo();
    bool redo();

protected:
    std::vector<Command::Ptr> nested;
    bool locked;
};
}
}


#endif // COMMAND_META_H
