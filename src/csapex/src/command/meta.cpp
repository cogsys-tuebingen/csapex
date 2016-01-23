/// HEADER
#include <csapex/command/meta.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex::command;

Meta::Meta(const std::string &type)
    : locked(false), type(type)
{
}

void Meta::init(Settings* settings, GraphFacade* graph_facade,
                ThreadPool* thread_pool, NodeFactory *node_factory)
{
    Command::init(settings, graph_facade, thread_pool, node_factory);
    for(Command::Ptr cmd : nested) {
        cmd->init(settings, graph_facade, thread_pool, node_factory);
    }
}

void Meta::accept(int level, std::function<void (int level, const Command &)> callback) const
{
    callback(level, *this);
    for(Command::Ptr cmd : nested) {
        cmd->accept(level+1, callback);
    }
}

std::string Meta::getType() const
{
    return type;
}

std::string Meta::getDescription() const
{
    return "";
}

void Meta::clear()
{
    apex_assert_hard(!locked);
    nested.clear();
}

void Meta::add(Command::Ptr cmd)
{
    apex_assert_hard(!locked);
    apex_assert_hard(cmd);

    if(initialized_) {
        cmd->init(settings_, root_, thread_pool_, node_factory_);
    }

    nested.push_back(cmd);
}

int Meta::commands() const
{
    return nested.size();
}

bool Meta::doExecute()
{
    locked = true;

    bool success = true;
    for(Command::Ptr cmd : nested) {
        bool s = Access::executeCommand(cmd);
        if(!s) {
            auto& command = *cmd;
            std::cerr << "command failed to execute! (" << typeid(command).name() << ")" << std::endl;
        }
        success &= s;
    }
    return success;
}

bool Meta::doUndo()
{
    for(auto it = nested.rbegin(); it != nested.rend(); ++it) {
        bool s = Access::undoCommand(*it);
        if(!s) {
            undo_later.push_back(*it);
        }
    }

    return true;
}

bool Meta::doRedo()
{
    bool success = true;
    for(Command::Ptr cmd : nested) {
        bool s = Access::redoCommand(cmd);
        success &= s;
    }
    return success;
}
