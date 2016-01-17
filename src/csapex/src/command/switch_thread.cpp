/// HEADER
#include <csapex/command/switch_thread.h>

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
    TaskGenerator* tg = getGraphFacade()->getTaskGenerator(uuid);

    auto group = getThreadPool()->getGroupFor(tg);
    old_id = group->id();

    if(id != ThreadGroup::PRIVATE_THREAD) {
        name = group->name();
    }
    getThreadPool()->addToGroup(tg, id);

    return true;
}

bool SwitchThread::doUndo()
{
    TaskGenerator* tg = getGraphFacade()->getTaskGenerator(uuid);

    getThreadPool()->addToGroup(tg, old_id);

    return true;
}

bool SwitchThread::doRedo()
{
    return doExecute();
}

