#ifndef COMMAND_H
#define COMMAND_H

/// COMPONENT
#include <csapex/command/command_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/csapex_command_export.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/factory/factory_fwd.h>

/// SYSTEM
#include <memory>
#include <vector>

namespace csapex
{

class Designer;

class CSAPEX_COMMAND_EXPORT Command
{

public:
    class CSAPEX_COMMAND_EXPORT Access {
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
    Command(const AUUID& graph_uuid);

    virtual void init(GraphFacade* graph_facade, CsApexCore& core, Designer* designer);
    virtual bool isUndoable() const;

    void setAfterSavepoint(bool save);
    bool isAfterSavepoint();

    void setBeforeSavepoint(bool save);
    bool isBeforeSavepoint();

    virtual void accept(int level, std::function<void (int level, const Command &)> callback) const;

    virtual std::string getType() const = 0;
    virtual std::string getDescription() const = 0;

protected:
    bool executeCommand(CommandPtr cmd);
    bool undoCommand(CommandPtr cmd);
    bool redoCommand(CommandPtr cmd);

    virtual bool doExecute() = 0;
    virtual bool doUndo() = 0;
    virtual bool doRedo() = 0;

    GraphFacade* getRoot();

    GraphFacade* getGraphFacade();
    Graph* getGraph();
    SubgraphNode* getSubgraphNode();

    NodeFactory* getNodeFactory();

    Designer* getDesigner();

    GraphFacade* getSubGraph(const UUID& graph_id);
    ThreadPool* getRootThreadPool();

protected:
    AUUID graph_uuid;
    CsApexCore* core_;

    GraphFacade* graph_facade_;

    Designer* designer_;

private:
    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;

    bool initialized_;
};

}

#endif // COMMAND_H
