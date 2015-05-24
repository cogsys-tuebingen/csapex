/// HEADER
#include <csapex/command/switch_thread.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

SwitchThread::SwitchThread(const UUID &node, int thread_id)
    : uuid(node), old_id(-1), id(thread_id)
{
}

std::string SwitchThread::getType() const
{
    return "SwitchThread";
}

std::string SwitchThread::getDescription() const
{
    std::stringstream ss;
    if(id == 0) {
        ss << "Switched to private thread for node " << uuid;
    } else {
        ss << "Switched thread for node " << uuid << " to " << name;
    }
    return ss.str();
}

bool SwitchThread::doExecute()
{
    NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    old_id = node_worker->getNodeState()->getThread();

    if(id == 0) {
        thread_pool_->usePrivateThreadFor(node_worker);
    } else {
        name = thread_pool_->getCustomGroup(id)->name;
        thread_pool_->addToGroup(node_worker, id);
    }

    return true;
}

bool SwitchThread::doUndo()
{
    NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    if(old_id == 0) {
        thread_pool_->usePrivateThreadFor(node_worker);
    } else {
        thread_pool_->addToGroup(node_worker, old_id);
    }
    return true;
}

bool SwitchThread::doRedo()
{
    return doExecute();
}

