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
    : before_save_point_(false), after_save_point_(false)
{
}

bool Command::Access::executeCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
{
    return Command::executeCommand(graph, widget_ctrl, cmd);
}

bool Command::Access::undoCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
{
    return Command::undoCommand(graph, widget_ctrl, cmd);
}

bool Command::Access::redoCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
{
    return Command::redoCommand(graph, widget_ctrl, cmd);
}

void Command::setGraph(Graph::Ptr graph)
{
    graph_ = graph;
}

Graph::Ptr Command::getGraph()
{
    return graph_;
}

void Command::setWidgetController(WidgetControllerPtr widget_ctrl)
{
    widget_ctrl_ = widget_ctrl;
}

bool Command::executeCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    cmd->widget_ctrl_ = widget_ctrl;
    apex_assert_hard(cmd->graph_);
    apex_assert_hard(cmd->widget_ctrl_);
    return cmd->doExecute();
}

bool Command::undoCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    cmd->widget_ctrl_ = widget_ctrl;
    apex_assert_hard(cmd->graph_);
    apex_assert_hard(cmd->widget_ctrl_);
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(GraphPtr graph, WidgetController::Ptr widget_ctrl, Command::Ptr cmd)
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
