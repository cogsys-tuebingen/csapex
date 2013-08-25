#ifndef COMMAND_H
#define COMMAND_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <vector>

namespace csapex
{

class Command
{
    friend class CommandDispatcher;
    friend class command::Meta;

public:
    typedef boost::shared_ptr<Command> Ptr;

public:
    Command();

    void setAfterSavepoint(bool save);
    bool isAfterSavepoint();

    void setBeforeSavepoint(bool save);
    bool isBeforeSavepoint();

protected:
    virtual bool execute() = 0;
    virtual bool undo() = 0;
    virtual bool redo() = 0;

    bool doExecute(Ptr other);
    bool doUndo(Ptr other);
    bool doRedo(Ptr other);

    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;
};

}

#endif // COMMAND_H
