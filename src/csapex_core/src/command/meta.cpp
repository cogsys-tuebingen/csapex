/// HEADER
#include <csapex/command/meta.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/core/csapex_core.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(Meta)

Meta::Meta(const AUUID& parent_uuid, const std::string& type, bool transaction) : CommandImplementation(parent_uuid), locked(false), transaction(transaction), type(type)
{
}

void Meta::init(GraphFacadeImplementation* root, CsApexCore& core)
{
    Command::init(root, core);
    for (Command::Ptr cmd : nested) {
        cmd->init(root, core);
    }
}

void Meta::accept(int level, std::function<void(int level, const Command&)> callback) const
{
    callback(level, *this);
    for (Command::Ptr cmd : nested) {
        cmd->accept(level + 1, callback);
    }
}

std::string Meta::getDescription() const
{
    return type;
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

    if (initialized_) {
        cmd->init(core_->getRoot().get(), *core_);
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

    if (transaction) {
        root_graph_facade_->getLocalGraph()->beginTransaction();
    }

    bool success = true;
    for (Command::Ptr cmd : nested) {
        bool s = Access::executeCommand(cmd);
        if (!s) {
            auto& command = *cmd;
            std::cerr << "command failed to execute! (" << typeid(command).name() << ")" << std::endl;
        }
        success &= s;
    }

    if (transaction) {
        root_graph_facade_->getLocalGraph()->finalizeTransaction();
    }

    return success;
}

bool Meta::doUndo()
{
    if (transaction) {
        root_graph_facade_->getLocalGraph()->beginTransaction();
    }

    for (auto it = nested.rbegin(); it != nested.rend(); ++it) {
        bool s = Access::undoCommand(*it);
        if (!s) {
            undo_later.push_back(*it);
        }
    }

    if (transaction) {
        root_graph_facade_->getLocalGraph()->finalizeTransaction();
    }

    return true;
}

bool Meta::doRedo()
{
    bool success = true;
    for (Command::Ptr cmd : nested) {
        bool s = Access::redoCommand(cmd);
        success &= s;
    }
    return success;
}

void Meta::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << nested;
    data << type;
    data << locked;
    data << transaction;
}

void Meta::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> nested;
    data >> type;
    data >> locked;
    data >> transaction;
}

void Meta::cloneData(const Meta& other)
{
    nested.clear();
    nested.reserve(other.nested.size());
    for (const CommandPtr& cmd : other.nested) {
        nested.emplace_back(cmd->cloneAs<Command>());
    }

    locked = other.locked;
    transaction = other.transaction;
    type = other.type;
}
