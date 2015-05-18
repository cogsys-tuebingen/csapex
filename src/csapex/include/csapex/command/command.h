#ifndef COMMAND_H
#define COMMAND_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

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
        static bool executeCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);
        static bool undoCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);
        static bool redoCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);
    };

public:
    typedef std::shared_ptr<Command> Ptr;

public:
    Command();

    void init(Settings* settings, GraphWorker *graph, ThreadPool* thread_pool, NodeFactory *node_factory);

    void setAfterSavepoint(bool save);
    bool isAfterSavepoint();

    void setBeforeSavepoint(bool save);
    bool isBeforeSavepoint();

    virtual void accept(int level, std::function<void (int level, const Command &)> callback) const;

    virtual std::string getType() const = 0;
    virtual std::string getDescription() const = 0;

protected:
    static bool executeCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);
    static bool undoCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);
    static bool redoCommand(GraphWorker* graph, ThreadPool* thread_pool, NodeFactory* node_factory, CommandPtr cmd);

    virtual bool doExecute() = 0;
    virtual bool doUndo() = 0;
    virtual bool doRedo() = 0;

protected:
    Settings* settings_;
    GraphWorker* graph_worker_;
    ThreadPool* thread_pool_;
    NodeFactory* node_factory_;

    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;
};

}

#endif // COMMAND_H
