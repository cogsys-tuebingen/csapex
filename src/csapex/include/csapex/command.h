#ifndef COMMAND_H
#define COMMAND_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <vector>

namespace csapex
{

class Graph;

namespace command {
class Meta;
}

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
    virtual bool execute(Graph& graph) = 0;
    virtual bool undo(Graph& graph) = 0;
    virtual bool redo(Graph& graph) = 0;

    bool doExecute(Graph& graph, Ptr other);
    bool doUndo(Graph& graph, Ptr other);
    bool doRedo(Graph& graph, Ptr other);

    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;
};

}

#endif // COMMAND_H
