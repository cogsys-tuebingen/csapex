/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/utility/assert.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/core/csapex_core.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

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

void Command::init(GraphFacadeImplementation *graph_facade, CsApexCore& core)
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
bool Command::isHidden() const
{
    return false;
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

GraphFacadeImplementation* Command::getRoot()
{
    GraphFacadeImplementation* gfl = dynamic_cast<GraphFacadeImplementation*>(root_graph_facade_);
    apex_assert_hard(gfl);
    return gfl;
}

GraphFacadeImplementation* Command::getGraphFacade()
{
    GraphFacade* gf = nullptr;

    if(graph_uuid.empty()) {
        gf = getRoot();
        apex_assert_hard(gf);

    } else if(getRoot()->getAbsoluteUUID() == graph_uuid) {
        gf = getRoot();
        apex_assert_hard(gf);

    } else {
        gf = getRoot()->getSubGraph(graph_uuid).get();
        apex_assert_hard(gf);
    }

    GraphFacadeImplementation* gfl = dynamic_cast<GraphFacadeImplementation*>(gf);
    apex_assert_hard(gfl);

    return gfl;
}

GraphImplementationPtr Command::getGraph()
{
    return getGraphFacade()->getLocalGraph();
}

SubgraphNodePtr Command::getSubgraphNode()
{
    return getGraphFacade()->getSubgraphNode();
}

NodeFactoryImplementation* Command::getNodeFactory()
{
    return core_->getNodeFactory().get();
}

GraphFacade* Command::getSubGraph(const UUID& graph_id)
{
    return getRoot()->getSubGraph(graph_id).get();
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
    data << graph_uuid;
}

void Command::deserialize(const SerializationBuffer& data)
{
    data >> graph_uuid;
}

std::shared_ptr<Clonable> Command::cloneRaw() const
{
    auto res = std::dynamic_pointer_cast<Command>(makeEmptyClone());
    res->cloneFrom(*this);
    return res;
}
