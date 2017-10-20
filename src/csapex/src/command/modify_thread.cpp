/// HEADER
#include <csapex/command/modify_thread.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/cpu_affinity.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(ModifyThread)

ModifyThread::ModifyThread(int thread_id, const std::string &name, std::vector<bool> cpu_affinity)
    : id(thread_id), name(name), affinity(cpu_affinity)
{
}

std::string ModifyThread::getDescription() const
{
    std::stringstream ss;
    ss << "Modifiy thread " << id;
    if(!name.empty()) {
        ss<< ", (name=" << name << ")";
    }

    if(!affinity.empty()) {
        ss << ", (affinity=";
        for(bool cpu : affinity) {
            ss << (int) cpu;
        }
        ss << ")";
    }
    return ss.str();
}

ThreadGroup* ModifyThread::getGroup()
{
    switch(id)
    {
    case ThreadGroup::PRIVATE_THREAD:
        throw std::runtime_error("cannot get private groups");
    default:
        return getRootThreadPool()->getGroup(id);
    }

}

bool ModifyThread::doExecute()
{
    if(id == ThreadGroup::PRIVATE_THREAD)  {
        // name cannot be changed
        old_affinity = getRootThreadPool()->getPrivateThreadGroupCpuAffinity();
        getRootThreadPool()->setPrivateThreadGroupCpuAffinity(affinity);

    } else {
        ThreadGroup* group = getGroup();

        old_name = group->getName();
        if(!name.empty()) {
            group->setName(name);
        }

        old_affinity = group->getCpuAffinity()->get();
        if(!affinity.empty()) {
            group->getCpuAffinity()->set(affinity);
        }
    }

    return true;
}

bool ModifyThread::doUndo()
{
    if(id == ThreadGroup::PRIVATE_THREAD)  {
        // name cannot be changed
        getRootThreadPool()->setPrivateThreadGroupCpuAffinity(old_affinity);

    } else {
        ThreadGroup* group = getGroup();

        if(!name.empty()) {
            group->setName(old_name);
        }

        if(!affinity.empty()) {
            group->getCpuAffinity()->set(old_affinity);
        }
    }
    return true;
}

bool ModifyThread::doRedo()
{
    return doExecute();
}



void ModifyThread::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << id;
    data << name;
    data << affinity;
}

void ModifyThread::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> id;
    data >> name;
    data >> affinity;
}
