/// HEADER
#include <csapex/command/modify_thread.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

ModifyThread::ModifyThread(int thread_id, const std::string &name)
    : id(thread_id), name(name)
{
}

std::string ModifyThread::getDescription() const
{
    std::stringstream ss;
    if(id == 0) {
        ss << "Modifiy thread " << id;
    }
    return ss.str();
}

bool ModifyThread::doExecute()
{
    auto group = getRootThreadPool()->getGroup(id);

    old_name = group->getName();
    group->setName(name);

    return true;
}

bool ModifyThread::doUndo()
{
    auto group = getRootThreadPool()->getGroup(id);

    group->setName(old_name);

    return true;
}

bool ModifyThread::doRedo()
{
    return doExecute();
}

