/// HEADER
#include <csapex/command/create_thread.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/model/node_runner.h>

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
    TaskGenerator* tg = graph_facade_->getTaskGenerator(uuid);

    auto group = thread_pool_->getGroupFor(tg);

    old_id = group->id();
    new_id = thread_pool_->createNewGroupFor(tg, name);

    return true;
}

bool CreateThread::doUndo()
{
    TaskGenerator* tg = graph_facade_->getTaskGenerator(uuid);

    thread_pool_->addToGroup(tg, old_id);

    return true;
}

bool CreateThread::doRedo()
{
    return doExecute();
}

