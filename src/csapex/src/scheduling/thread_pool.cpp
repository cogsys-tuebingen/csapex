/// HEADER
#include <csapex/scheduling/thread_pool.h>

/// COMPONENT
#include <csapex/utility/yaml_io.hpp>
#include <csapex/utility/thread.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/scheduling/task.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/scheduling/timed_queue.h>
#include <csapex/utility/cpu_affinity.h>

/// SYSTEM
#include <set>
#include <unordered_map>
#include <iostream>

using namespace csapex;

ThreadPool::ThreadPool(ExceptionHandler& handler, bool enable_threading, bool grouping)
    : handler_(handler),
      timed_queue_(new TimedQueue),
      enable_threading_(enable_threading), grouping_(grouping),
      private_group_cpu_affinity_(new CpuAffinity)
{
    setup();
}

ThreadPool::ThreadPool(Executor* parent, ExceptionHandler& handler, bool enable_threading, bool grouping)
    : handler_(handler), enable_threading_(enable_threading), grouping_(grouping),
      private_group_cpu_affinity_(new CpuAffinity)
{
    setup();
    parent->addChild(this);
}
void ThreadPool::setup()
{
    default_group_ = std::make_shared<ThreadGroup>(timed_queue_,
                                                   handler_,
                                                   ThreadGroup::DEFAULT_GROUP_ID, "default");

    groups_.push_back(default_group_);

    observe(default_group_->end_step, [this]() {
        checkIfStepIsDone();
    });

    observe(private_group_cpu_affinity_->affinity_changed, [this](const CpuAffinity*){
        private_group_cpu_affinity_changed();
    });

    setPause(false);
    setSteppingMode(false);
}

ThreadPool::~ThreadPool()
{
    group_assignment_.clear();
}


bool ThreadPool::isThreadingEnabled() const
{
    return enable_threading_;
}
bool ThreadPool::isGroupingEnabled() const
{
    return grouping_;
}

void ThreadPool::performStep()
{
    if(!default_group_->isEmpty() || groups_.size() > 1) {
        for(auto g : groups_) {
            g->step();
        }
    }
}

void ThreadPool::start()
{
    for(auto g : groups_) {
        g->start();
    }
}

void ThreadPool::stop()
{
    for(auto g : groups_) {
        g->stop();
    }
    group_assignment_.clear();
    groups_.clear();
    default_group_.reset();
    apex_assert_hard(group_assignment_.empty());
    apex_assert_hard(groups_.empty());
}

bool ThreadPool::isRunning() const
{
    return default_group_->isRunning();
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


void ThreadPool::steppingChanged(bool step)
{
    for(auto g : groups_) {
        g->setSteppingMode(step);
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

    group_connection_[schedulable] = schedulable->stepping_enabled.connect([this](){
        checkIfStepCanBeDone();
    });
}

void ThreadPool::checkIfStepCanBeDone()
{
    for(auto group : groups_) {
        if(!group->canStartStepping()) {
            return;
        }
    }

    stepping_enabled();
}

void ThreadPool::remove(TaskGenerator *task)
{
    task->detach();
    group_assignment_.erase(task);
    group_connection_.erase(task);
}

std::size_t ThreadPool::getGroupCount() const
{
    return groups_.size();
}

ThreadGroup *ThreadPool::getGroupAt(std::size_t pos)
{
    return groups_.at(pos).get();
}

std::vector<ThreadGroupPtr> ThreadPool::getGroups()
{
    return groups_;
}

ThreadGroup* ThreadPool::getDefaultGroup()
{
    return default_group_.get();
}

ThreadGroup* ThreadPool::getGroup(int id)
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
        ThreadGroup* old_group = nullptr;

        auto pos = group_assignment_.find(task);
        if(pos != group_assignment_.end()) {
            old_group = pos->second;
            group_assignment_.erase(pos);
        }

        task->detach();
        task->assignToScheduler(group);
        group_assignment_[task] = group;

        if(old_group) {
            if(old_group->id() == ThreadGroup::PRIVATE_THREAD) {
                private_group_connections_[old_group].clear();
                removeGroup(old_group);
            }
        }
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
        ThreadGroupPtr group = std::make_shared<ThreadGroup>(timed_queue_,
                                                             handler_, ThreadGroup::PRIVATE_THREAD,
                                                             task->getUUID().getShortName());

        group->getCpuAffinity()->set(private_group_cpu_affinity_->get());

        group->setPause(isPaused());

        groups_.push_back(group);
        group->end_step.connect([this]() {
            checkIfStepIsDone();
        });

        assignGeneratorToGroup(task, group.get());

        ThreadGroupWeakPtr group_weak = group;
        private_group_connections_[group.get()].push_back(
                    group->getCpuAffinity()->affinity_changed.connect([this](const CpuAffinity* affinity) {
            private_group_cpu_affinity_->set(affinity->get());
        }));

        private_group_connections_[group.get()].push_back(
                    private_group_cpu_affinity_changed.connect([this, group_weak](){
                        if(ThreadGroupPtr group = group_weak.lock()) {
                            group->getCpuAffinity()->set(private_group_cpu_affinity_->get());
                        }
                    }));

        if(isRunning()) {
            group->start();
        }

        group_created(group);
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
        if(group->getName() == name) {
            assignGeneratorToGroup(task, group.get());
            return group->id();
        }
    }

    ThreadGroup* group = createGroup(name);

    assignGeneratorToGroup(task, group);


    return group->id();
}

ThreadGroup* ThreadPool::createGroup(const std::string &name, int id)
{
    ThreadGroupPtr group;
    if(id > 0 ) {
        group = std::make_shared<ThreadGroup>(timed_queue_, handler_, id, name);
    } else {
        group = std::make_shared<ThreadGroup>(timed_queue_, handler_, name);
    }
    group->setPause(isPaused());

    groups_.push_back(group);
    group->end_step.connect([this]() {
        checkIfStepIsDone();
    });

    if(isRunning()) {
        group->start();
    }

    group_created(group);

    return group.get();
}

void ThreadPool::removeGroup(int id)
{
    for(auto it = groups_.begin(); it != groups_.end(); ++it ) {
        ThreadGroupPtr group = *it;
        if(group->id() == id) {
            apex_assert_hard(group->isEmpty());
            groups_.erase(it);

            group_removed(group);
            return;
        }
    }
}

void ThreadPool::removeGroup(ThreadGroup *group)
{
    for(auto it = groups_.begin(); it != groups_.end(); ++it ) {
        if(it->get() == group) {
            apex_assert_hard(group->isEmpty());
            group_removed(*it);
            groups_.erase(it);

            return;
        }
    }
}



bool ThreadPool::isStepDone()
{
    //TRACE std::cerr << " TP CHECK =========== " << std::endl;
    for(auto group : groups_) {
        if(!group->isStepDone()) {
            return false;
        }
    }

    return true;
}

std::string ThreadPool::nextName()
{
    std::stringstream name;
    name << "Thread " << ThreadGroup::nextId();
    return name.str();
}

void ThreadPool::setPrivateThreadGroupCpuAffinity(const std::vector<bool>& affinity)
{
    private_group_cpu_affinity_->set(affinity);
}

std::vector<bool> ThreadPool::getPrivateThreadGroupCpuAffinity() const
{
    return private_group_cpu_affinity_->get();
}

void ThreadPool::saveSettings(YAML::Node& node)
{
    YAML::Node threads(YAML::NodeType::Map);

    YAML::Node groups;

    for(std::size_t i = 0, total =  groups_.size(); i < total; ++i) {
        YAML::Node group;
        int id = groups_[i]->id();
        group["id"] =  id;
        if(id != ThreadGroup::PRIVATE_THREAD ) {
            group["name"] =  groups_[i]->getName();
            groups_[i]->saveSettings(group);
            groups.push_back(group);
        }
    }
    threads["groups"] = groups;
    threads["private_affinity"] = private_group_cpu_affinity_->get();

    YAML::Node assignments;
    for(std::map<TaskGenerator*, ThreadGroup*>::const_iterator it = group_assignment_.begin();
        it != group_assignment_.end(); ++it)
    {
        YAML::Node assignment;
        TaskGenerator* tg = it->first;
        assignment["uuid"] = tg->getUUID().getAbsoluteUUID().getFullName();
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
        const YAML::Node& private_affinity = threads["private_affinity"];
        if(private_affinity.IsDefined()){
            std::vector<bool> a = private_affinity.as<std::vector<bool>>();
            private_group_cpu_affinity_->set(a);
        }

        const YAML::Node& groups = threads["groups"];
        if(groups.IsDefined()) {
            for(std::size_t i = 0, total = groups.size(); i < total; ++i) {
                const YAML::Node& group = groups[i];

                int group_id = group["id"].as<int>();

                ThreadGroup* group_ptr = nullptr;
                if(group_id >= ThreadGroup::MINIMUM_THREAD_ID) {
                    std::string group_name = group["name"].as<std::string>();

                    auto g = std::make_shared<ThreadGroup>(timed_queue_, handler_, group_id, group_name);
                    g->setPause(isPaused());

                    groups_.push_back(g);
                    g->end_step.connect([this]() {
                        checkIfStepIsDone();
                    });

                    group_created(g);

                    group_ptr = g.get();

                } else if(group_id == ThreadGroup::DEFAULT_GROUP_ID) {
                    group_ptr = getDefaultGroup();
                } else if(group_id == ThreadGroup::PRIVATE_THREAD) {
                    // private thread settings are not supported
                }

                if(group_ptr) {
                    group_ptr->loadSettings(group);
                }
            }
        }


        std::unordered_map<AUUID, int, UUID::Hasher> assignment_map;

        const YAML::Node& assignments = threads["assignments"];
        if(assignments.IsDefined()) {
            for(std::size_t i = 0, total = assignments.size(); i < total; ++i) {
                const YAML::Node& assignment = assignments[i];

                std::string uuid_str = assignment["uuid"].as<std::string>();
                AUUID uuid(UUIDProvider::makeUUID_without_parent(uuid_str));
                int id = assignment["id"].as<int>();

                assignment_map[uuid] = id;
            }
        }

        std::vector<TaskGenerator*> tasks;
        for(auto it = group_assignment_.begin(); it != group_assignment_.end(); ++it) {
            tasks.push_back(it->first);
        }
        for(TaskGenerator* task : tasks) {
            if(assignment_map.find(task->getUUID().getAbsoluteUUID()) != assignment_map.end()) {
                int id = assignment_map.at(task->getUUID().getAbsoluteUUID());
                addToGroup(task, id);
            }
        }
    }
}
