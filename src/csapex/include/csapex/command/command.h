#ifndef COMMAND_H
#define COMMAND_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <vector>

/// FORWARD DECLARATION
class QTreeWidgetItem;

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
        static bool executeCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);
        static bool undoCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);
        static bool redoCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);
    };

public:
    typedef boost::shared_ptr<Command> Ptr;

public:
    Command();

    void init(Settings* settings, GraphPtr graph, WidgetControllerPtr widget_ctrl);

    void setAfterSavepoint(bool save);
    bool isAfterSavepoint();

    void setBeforeSavepoint(bool save);
    bool isBeforeSavepoint();

    virtual QTreeWidgetItem* createDebugInformation() const;

    virtual std::string getType() const = 0;
    virtual std::string getDescription() const = 0;

protected:
    static bool executeCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);
    static bool undoCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);
    static bool redoCommand(GraphPtr graph, WidgetControllerPtr widget_ctrl, CommandPtr cmd);

    virtual bool doExecute() = 0;
    virtual bool doUndo() = 0;
    virtual bool doRedo() = 0;

protected:
    Settings* settings_;
    GraphPtr graph_;
    WidgetControllerPtr widget_ctrl_;

    static std::vector<Command::Ptr> undo_later;

    bool before_save_point_;
    bool after_save_point_;
};

}

#endif // COMMAND_H
