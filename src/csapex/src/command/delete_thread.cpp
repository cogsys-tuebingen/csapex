/// HEADER
#include <csapex/command/delete_thread.h>

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

using namespace csapex;
using namespace csapex::command;


DeleteThread::DeleteThread(int thread_id)
    : id(thread_id)
{
}

std::string DeleteThread::getDescription() const
{
    std::stringstream ss;
    ss << "delete thread " << id;
    return ss.str();
}

bool DeleteThread::doExecute()
{
    ThreadPool* thread_pool = getRootThreadPool();

    ThreadGroup* group = thread_pool->getGroup(id);
    apex_assert_hard(group->isEmpty());

    name = group->getName();

    thread_pool->removeGroup(id);

    return true;
}

bool DeleteThread::doUndo()
{
    ThreadPool* thread_pool = getRootThreadPool();

    thread_pool->createGroup(name, id);

    return true;
}

bool DeleteThread::doRedo()
{
    return doExecute();
}

