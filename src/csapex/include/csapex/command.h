#ifndef COMMAND_H
#define COMMAND_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <vector>

namespace csapex
{

namespace command {
class Meta;
}

class Command
{
    friend class BoxManager;
    friend class command::Meta;

public:
    typedef boost::shared_ptr<Command> Ptr;

public:
    Command();

protected:
    virtual void execute() = 0;
    virtual bool undo() = 0;
    virtual void redo() = 0;

    void doExecute(Ptr other);
    bool doUndo(Ptr other);
    void doRedo(Ptr other);

    static std::vector<Command::Ptr> undo_later;
};

}

#endif // COMMAND_H
