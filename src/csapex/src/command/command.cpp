/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <QTreeWidgetItem>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : settings_(NULL), before_save_point_(false), after_save_point_(false)
{
}

bool Command::Access::executeCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    return Command::executeCommand(graph, widget_ctrl, cmd);
}

bool Command::Access::undoCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    return Command::undoCommand(graph, widget_ctrl, cmd);
}

bool Command::Access::redoCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    return Command::redoCommand(graph, widget_ctrl, cmd);
}

void Command::init(Settings *settings, Graph* graph, WidgetController* widget_ctrl)
{
    settings_ = settings;
    graph_ = graph;
    widget_ctrl_ = widget_ctrl;
}

bool Command::executeCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    apex_assert_hard(graph);
    apex_assert_hard(widget_ctrl);
    cmd->graph_ = graph;
    cmd->widget_ctrl_ = widget_ctrl;
    return cmd->doExecute();
}

bool Command::undoCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    apex_assert_hard(graph);
    apex_assert_hard(widget_ctrl);
    cmd->graph_ = graph;
    cmd->widget_ctrl_ = widget_ctrl;
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(Graph* graph, WidgetController* widget_ctrl, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    cmd->widget_ctrl_ = widget_ctrl;
    apex_assert_hard(cmd->graph_);
    apex_assert_hard(cmd->widget_ctrl_);
    return cmd->doRedo();
}

void Command::setAfterSavepoint(bool save)
{
    after_save_point_ = save;
}

bool Command::isAfterSavepoint()
{
    return after_save_point_;
}

void Command::setBeforeSavepoint(bool save)
{
    before_save_point_ = save;
}

bool Command::isBeforeSavepoint()
{
    return before_save_point_;
}

QTreeWidgetItem* Command::createDebugInformation() const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, getType().c_str());
    tl->setText(1, getDescription().c_str());
    return tl;
}
