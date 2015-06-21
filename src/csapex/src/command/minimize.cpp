/// HEADER
#include <csapex/command/minimize.h>

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

Minimize::Minimize(const UUID &node, bool mini)
    : uuid(node), mini(mini), executed(false)
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

    bool is_mini = node_worker->getNodeState()->isMinimized();

    if(is_mini != mini) {
        node_worker->getNodeState()->setMinimized(mini);
        executed = true;
    } else {
        executed = false;
    }

    return true;
}

bool Minimize::doUndo()
{
    if(executed) {
        NodeWorker* node_worker = graph_->findNodeWorker(uuid);
        apex_assert_hard(node_worker);

        node_worker->getNodeState()->setMinimized(!mini);
    }
    return true;
}

bool Minimize::doRedo()
{
    return doExecute();
}

