/// HEADER
#include <csapex/scheduling/thread_pool.h>

/// COMPONENT
#include <csapex/utility/yaml_io.hpp>
#include <csapex/utility/thread.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/scheduling/task.h>
#include <csapex/scheduling/task_generator.h>

/// SYSTEM
#include <set>
#include <unordered_map>

using namespace csapex;

ThreadPool::ThreadPool(bool enable_threading, bool grouping, bool paused)
    : enable_threading_(enable_threading), grouping_(grouping)
{
    default_group_ = std::make_shared<ThreadGroup>(ThreadGroup::DEFAULT_GROUP_ID, "default", paused);
}

void ThreadPool::stop()
{
    for(auto g : groups_) {
        g->stop();
    }
}

void ThreadPool::clear()
{
    bool p = isPaused();
    setPause(true);
    for(auto g : groups_) {
        g->clear();
    }
    setPause(p);
}

void ThreadPool::pauseChanged(bool pause)
{
    for(auto g : groups_) {
        g->setPause(pause);
    }
}


void ThreadPool::add(TaskGenerator *schedulable)
{
    if(enable_threading_) {
        if(!grouping_) {
            usePrivateThreadFor(schedulable);
        } else {
            // if we use grouping -> wait for structure change
            useDefaultThreadFor(schedulable);
        }
    } else {
        // this is the one thread to use when threading is disabled
        // we need this to make the ui responsive
        useDefaultThreadFor(schedulable);
    }
}

void ThreadPool::remove(TaskGenerator *task)
{
    task->detach();
    group_assignment_.erase(task);
}

std::vector<ThreadGroupPtr> ThreadPool::getGroups()
{
    return groups_;
}

ThreadGroup *ThreadPool::getGroup(int id)
{
    for(auto group : groups_) {
        if(group->id() == id) {
            return group.get();
        }
    }

    throw std::runtime_error("group doesn't exist");
}

ThreadGroup* ThreadPool::getGroupFor(TaskGenerator* generator)
{
    auto pos = group_assignment_.find(generator);
    if(pos == group_assignment_.end()) {
        throw std::runtime_error("group doesn't exist");
    } else {
        return pos->second;
    }
}

void ThreadPool::assignGeneratorToGroup(TaskGenerator *task, ThreadGroup *group)
{
    if(!isInGroup(task, group->id())) {
        auto pos = group_assignment_.find(task);
        if(pos != group_assignment_.end()) {
            group_assignment_.erase(pos);
        }

        task->detach();
        task->assignToScheduler(group);
        group_assignment_[task] = group;
    }
}

bool ThreadPool::isInPrivateThread(TaskGenerator *task) const
{
    return isInGroup(task, ThreadGroup::PRIVATE_THREAD);
}

bool ThreadPool::isInGroup(TaskGenerator *task, int id) const
{
    auto pos = group_assignment_.find(task);
    if(pos != group_assignment_.end()) {
        return pos->second->id() == id;
    }
    return false;
}

void ThreadPool::usePrivateThreadFor(TaskGenerator *task)
{
    if(!isInPrivateThread(task)) {
        auto group = std::make_shared<ThreadGroup>(ThreadGroup::PRIVATE_THREAD, task->getUUID().getShortName(), isPaused());
        groups_.push_back(group);

        assignGeneratorToGroup(task, group.get());
    }
}

void ThreadPool::useDefaultThreadFor(TaskGenerator *task)
{
    assignGeneratorToGroup(task, default_group_.get());
}

void ThreadPool::addToGroup(TaskGenerator *task, int group_id)
{
    if(group_id == ThreadGroup::PRIVATE_THREAD) {
        usePrivateThreadFor(task);

    } else if(group_id == ThreadGroup::DEFAULT_GROUP_ID) {
        useDefaultThreadFor(task);

    } else if(group_id >= ThreadGroup::MINIMUM_THREAD_ID) {
        for(auto g : groups_) {
            if(g->id() == group_id) {
                assignGeneratorToGroup(task, g.get());
                return;
            }
        }
    }
}


int ThreadPool::createNewGroupFor(TaskGenerator* task, const std::string &name)
{
    for(auto group : groups_) {
        if(group->name() == name) {
            assignGeneratorToGroup(task, group.get());
            return group->id();
        }
    }

    ThreadGroupPtr group = std::make_shared<ThreadGroup>(name, isPaused());
    groups_.push_back(group);

    assignGeneratorToGroup(task, group.get());

    return group->id();
}

//void ThreadPool::clearGroup(ThreadGroup* g)
//{
//    for(auto it = groups_.begin(); it != groups_.end(); ++it) {
//        auto group = *it;
//        if(group.get() == g) {
//            if(group->isEmpty()) {
//                group->stop();
////                groups_.erase(it);
//            }
//            return;
//        }
//    }
//    throw std::runtime_error("attempted to delete non-existing thread group");
//}

std::string ThreadPool::nextName()
{
    std::stringstream name;
    name << "Thread " << ThreadGroup::nextId();
    return name.str();
}

void ThreadPool::saveSettings(YAML::Node& node)
{
    YAML::Node threads(YAML::NodeType::Map);

    YAML::Node groups;
    for(std::size_t i = 0, total =  groups_.size(); i < total; ++i) {
        YAML::Node group;
        group["id"] =  groups_[i]->id();
        group["name"] =  groups_[i]->name();
        groups.push_back(group);
    }
    threads["groups"] = groups;

    YAML::Node assignments;
    for(std::map<TaskGenerator*, ThreadGroup*>::const_iterator it = group_assignment_.begin();
        it != group_assignment_.end(); ++it)
    {
        YAML::Node assignment;
        assignment["uuid"] = it->first->getUUID();
        assignment["id"] = it->second->id();
        assignments.push_back(assignment);
    }
    threads["assignments"] = assignments;

    node["threads"] = threads;
}

void ThreadPool::loadSettings(YAML::Node& node)
{
    const YAML::Node& threads = node["threads"];
    if(threads.IsDefined()) {
        const YAML::Node& groups = threads["groups"];
        if(groups.IsDefined()) {
            for(std::size_t i = 0, total = groups.size(); i < total; ++i) {
                const YAML::Node& group = groups[i];

                int group_id = group["id"].as<int>();

                if(group_id >= ThreadGroup::MINIMUM_THREAD_ID) {
                    std::string group_name = group["name"].as<std::string>();
                    groups_.emplace_back(std::make_shared<ThreadGroup>(group_id, group_name, isPaused()));
                }
            }
        }


        std::unordered_map<UUID, int, UUID::Hasher> assignment_map;

        const YAML::Node& assignments = threads["assignments"];
        if(assignments.IsDefined()) {
            for(std::size_t i = 0, total = assignments.size(); i < total; ++i) {
                const YAML::Node& assignment = assignments[i];

                UUID uuid = assignment["uuid"].as<UUID>();
                int id = assignment["id"].as<int>();

                assignment_map[uuid] = id;
            }
        }

        for(auto it = group_assignment_.begin(); it != group_assignment_.end(); ++it) {
            TaskGenerator* task = it->first;
            if(assignment_map.find(task->getUUID()) != assignment_map.end()) {
                int id = assignment_map[task->getUUID()];
                addToGroup(task, id);
            }
        }
    }
}
