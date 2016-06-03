/// HEADER
#include <csapex/command/meta.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <iostream>

using namespace csapex::command;

Meta::Meta(const AUUID &parent_uuid, const std::string &type)
    : Command(parent_uuid), locked(false), type(type)
{
}

void Meta::init(GraphFacade* root, CsApexCore& core, Designer* designer)
{
    Command::init(root, core, designer);
    for(Command::Ptr cmd : nested) {
        cmd->init(root, core, designer);
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
        cmd->init(core_->getRoot().get(), *core_, designer_);
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
