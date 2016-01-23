#ifndef COMMAND_H
#define COMMAND_H

/// COMPONENT
#include <csapex/command/command_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/utility/uuid.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/factory/factory_fwd.h>

/// SYSTEM
#include <memory>
#include <vector>

namespace csapex
{

class Command
{

public:
    class Access {
        friend class Group;
        friend class CommandDispatcher;
        friend class command::Meta;

    private:
        static bool executeCommand(CommandPtr cmd);
        static bool undoCommand(CommandPtr cmd);
        static bool redoCommand(CommandPtr cmd);
    };

    friend class command::Meta;

public:
    typedef std::shared_ptr<Command> Ptr;

public:
    Command();

    virtual void init(Settings* settings, GraphFacade* graph_facade, ThreadPool* thread_pool, NodeFactory *node_factory);

    void setAfterSavepoint(bool save);
    bool isAfterSavepoint();

    void setBeforeSavepoint(bool save);
    bool isBeforeSavepoint();

    virtual void accept(int level, std::function<void (int level, const Command &)> callback) const;

    virtual std::string getType() const = 0;
    virtual std::string getDescription() const = 0;

protected:
    static bool executeCommand(CommandPtr cmd);
    static bool undoCommand(CommandPtr cmd);
    static bool redoCommand(CommandPtr cmd);

    virtual bool doExecute() = 0;
    virtual bool doUndo() = 0;
    virtual bool doRedo() = 0;

    GraphFacade* getRoot();
    GraphFacade* getSubGraph(const UUID& graph_id);
    Graph* getRootGraph();
    ThreadPool* getRootThreadPool();

protected:
    Settings* settings_;
    NodeFactory* node_factory_;

private:
    GraphFacade* root_;
    ThreadPool* thread_pool_;

    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;

    bool initialized_;
};

}

#endif // COMMAND_H
