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

protected:
    void execute();
    bool undo();
    void redo();

protected:
    std::vector<Command::Ptr> nested;
    bool locked;
};
}
}


#endif // COMMAND_META_H
