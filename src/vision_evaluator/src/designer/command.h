#ifndef COMMAND_H
#define COMMAND_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

namespace vision_evaluator {

class Command
{
    friend class BoxManager;

public:
    typedef boost::shared_ptr<Command> Ptr;

public:
    Command();

protected:
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual void redo() = 0;
};

}

#endif // COMMAND_H
