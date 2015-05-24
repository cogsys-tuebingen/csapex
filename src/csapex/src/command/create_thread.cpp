/// HEADER
#include <csapex/command/create_thread.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/scheduling/thread_pool.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;


CreateThread::CreateThread(const UUID &node, const std::string& name)
    : uuid(node), name(name), old_id(-1), new_id(-1)
{
}

std::string CreateThread::getType() const
{
    return "CreateThread";
}

std::string CreateThread::getDescription() const
{
    std::stringstream ss;
    ss << "created thread for " << uuid << " with name " << name;
    return ss.str();
}

bool CreateThread::doExecute()
{
    NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    old_id = node_worker->getNodeState()->getThread();
    new_id = thread_pool_->createNewGroupFor(node_worker, name);

    return true;
}

bool CreateThread::doUndo()
{
    NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorker(uuid);
    apex_assert_hard(node_worker);

    if(old_id == 0) {
        thread_pool_->usePrivateThreadFor(node_worker);
    } else {
        thread_pool_->addToGroup(node_worker, old_id);
    }
    thread_pool_->deleteGroup(new_id);

    return true;
}

bool CreateThread::doRedo()
{
    return doExecute();
}

