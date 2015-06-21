/// HEADER
#include <csapex/command/flip_sides.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

FlipSides::FlipSides(const UUID &node)
    : uuid(node)
{
}

std::string FlipSides::getType() const
{
    return "FlipSides";
}

std::string FlipSides::getDescription() const
{
    std::stringstream ss;
    ss << "flipped sides of " << uuid;
    return ss.str();
}

bool FlipSides::doExecute()
{
    NodeWorker* node_worker = graph_->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    bool flip = !node_worker->getNodeState()->isFlipped();
    node_worker->getNodeState()->setFlipped(flip);

    return true;
}

bool FlipSides::doUndo()
{
    return doExecute();
}

bool FlipSides::doRedo()
{
    return doExecute();
}

