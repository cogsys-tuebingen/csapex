/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>
#include <csapex/core/csapex_core.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command(const AUUID &parent_uuid)
    : Command()
{
    graph_uuid = parent_uuid;
}

Command::Command()
    : core_(nullptr), root_graph_facade_(nullptr),
      before_save_point_(false), after_save_point_(false),
      initialized_(false)
{

}

bool Command::Access::executeCommand(Command::Ptr cmd)
{
    return cmd->executeCommand(cmd);
}

bool Command::Access::undoCommand(Command::Ptr cmd)
{
    return cmd->undoCommand(cmd);
}

bool Command::Access::redoCommand(Command::Ptr cmd)
{
    return cmd->redoCommand(cmd);
}

void Command::init(GraphFacade* graph_facade, CsApexCore& core)
{
    apex_assert_hard(graph_facade);

    root_graph_facade_ = graph_facade;

    core_ = &core;

    initialized_ = true;
}

bool Command::isUndoable() const
{
    return true;
}

bool Command::executeCommand(Command::Ptr cmd)
{
    cmd->init(getRoot(), *core_);

    return cmd->doExecute();
}

bool Command::undoCommand(Command::Ptr cmd)
{
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(Command::Ptr cmd)
{
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

void Command::accept(int level, std::function<void (int level, const Command &)> callback) const
{
    callback(level, *this);
}

GraphFacade* Command::getRoot()
{
    return root_graph_facade_;
}

GraphFacade* Command::getGraphFacade()
{
    GraphFacade* gf = nullptr;

    if(graph_uuid.empty()) {
        gf = getRoot();
        apex_assert_hard(gf);

    } else if(getRoot()->getAbsoluteUUID() == graph_uuid) {
        gf = getRoot();
        apex_assert_hard(gf);

    } else {
        gf = getRoot()->getSubGraph(graph_uuid);
        apex_assert_hard(gf);
    }
    return gf;
}

Graph* Command::getGraph()
{
    return getGraphFacade()->getGraph();
}

SubgraphNode* Command::getSubgraphNode()
{
    return getGraphFacade()->getSubgraphNode();
}

NodeFactory* Command::getNodeFactory()
{
    return &core_->getNodeFactory();
}

GraphFacade* Command::getSubGraph(const UUID& graph_id)
{
    return getRoot()->getSubGraph(graph_id);
}

ThreadPool* Command::getRootThreadPool()
{
    return core_->getThreadPool().get();
}

uint8_t Command::getPacketType() const
{
    return PACKET_TYPE_ID;
}


void Command::serialize(SerializationBuffer &data) const
{
    throw std::runtime_error("command is not serializable");
}

void Command::deserialize(SerializationBuffer& data)
{
    throw std::runtime_error("command is not serializable");
}
