/// HEADER
#include <csapex/command/minimize.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

Minimize::Minimize(const UUID &node)
    : uuid(node), mini(true)
{
}

std::string Minimize::getType() const
{
    return "Minimize";
}

std::string Minimize::getDescription() const
{
    std::stringstream ss;
    ss << ( mini ? "min" : "max" ) << "imized" << uuid;
    return ss.str();
}

bool Minimize::doExecute()
{
    NodeWorker* node_worker = graph_->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    mini = !node_worker->getNodeState()->isMinimized();
    node_worker->getNodeState()->setMinimized(mini);

    return true;
}

bool Minimize::doUndo()
{
    return doExecute();
}

bool Minimize::doRedo()
{
    return doExecute();
}

