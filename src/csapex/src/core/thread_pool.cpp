/// HEADER
#include <csapex/core/thread_pool.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node_worker.h>
#include <csapex/core/csapex_core.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/utility/thread.h>

/// SYSTEM
#include <set>
#include <QVariant>

using namespace csapex;

ThreadPool::ThreadPool(Graph *graph, bool enable_threading, bool grouping)
    : next_id(MINIMUM_THREAD_ID), graph_(graph), enable_threading_(enable_threading), grouping_(grouping)
{
    connect(graph_, SIGNAL(nodeAdded(NodeWorkerPtr)), this, SLOT(nodeAdded(NodeWorkerPtr)));
    connect(graph_, SIGNAL(structureChanged(Graph*)), this, SLOT(structureChanged()));
}

std::vector<ThreadPool::Group> ThreadPool::getCustomGroups()
{
    return custom_groups_;
}

ThreadPool::Group ThreadPool::getCustomGroup(int id)
{
    for(std::vector<Group>::iterator it = custom_groups_.begin();
         it != custom_groups_.end(); ++it) {
         Group& group = *it;
         if(group.id == id) {
             return group;
         }
    }

    throw std::runtime_error("group doesn't exist");
}

void ThreadPool::saveSettings(YAML::Node& node)
{
    YAML::Node threads(YAML::NodeType::Map);

    YAML::Node groups;
    for(std::size_t i = 0, total =  custom_groups_.size(); i < total; ++i) {
        YAML::Node group;
        group["id"] =  custom_groups_[i].id;
        group["name"] =  custom_groups_[i].name;
        groups.push_back(group);
    }
    threads["groups"] = groups;

    YAML::Node assignments;
    for(std::map<NodeWorker*, Group*>::const_iterator it = custom_group_assignment_.begin();
        it != custom_group_assignment_.end(); ++it)
    {
        YAML::Node assignment;
        assignment["uuid"] = it->first->getUUID();
        assignment["id"] = it->second->id;
        assignments.push_back(assignment);
    }
    threads["assignments"] = assignments;
    threads["next_id"] = next_id;

    node["threads"] = threads;
}

void ThreadPool::loadSettings(YAML::Node& node)
{
    const YAML::Node& threads = node["threads"];
    if(threads.IsDefined()) {
        next_id = threads["next_id"].as<int>();

        const YAML::Node& groups = threads["groups"];
        if(groups.IsDefined()) {
            for(std::size_t i = 0, total = groups.size(); i < total; ++i) {
                const YAML::Node& group = groups[i];

                int group_id = group["id"].as<int>();
                std::string group_name = group["name"].as<std::string>();
                custom_groups_.push_back(Group(group_id, group_name));

                custom_group_threads_[group_id] = setupThread(group_id, true, group_name);
            }
        }

        const YAML::Node& assignments = threads["assignments"];
        if(assignments.IsDefined()) {
            for(std::size_t i = 0, total = assignments.size(); i < total; ++i) {
                const YAML::Node& assignment = assignments[i];

                UUID uuid = assignment["uuid"].as<UUID>();

                NodeWorker* worker = graph_->findNodeWorkerNoThrow(uuid);
                if(worker) {
                    int id = assignment["id"].as<int>();
                    switchToThread(worker, id);
                }
            }
        }
    }
}

void ThreadPool::usePrivateThreadFor(NodeWorker *worker)
{
    private_thread_[worker] = true;

    custom_group_assignment_.erase(worker);

    QThread* thread = setupThread(PRIVATE_THREAD, false, worker->getUUID().getShortName());
    worker->triggerSwitchThreadRequest(thread, PRIVATE_THREAD);
}

void ThreadPool::useDefaultThreadFor(NodeWorker* node_worker)
{
    private_thread_[node_worker] = false;

    static QThread* thread = nullptr;
    if(!thread) {
        thread = new QThread;
        thread->start();
    }

    node_worker->triggerSwitchThreadRequest(thread, UNDEFINED_THREAD);
}

void ThreadPool::switchToThread(NodeWorker *worker, int group_id)
{
    private_thread_[worker] = false;

    for(std::size_t i = 0, total =  custom_groups_.size(); i < total; ++i) {
        if(custom_groups_[i].id == group_id) {
            custom_group_assignment_[worker] = &custom_groups_[i];
            break;
        }
    }

    worker->triggerSwitchThreadRequest(custom_group_threads_[group_id], group_id);
}


int ThreadPool::createNewThreadGroupFor(NodeWorker* worker, const std::string &name)
{
    private_thread_[worker] = false;

    custom_groups_.push_back(Group(next_id++, name));
    Group& group = *custom_groups_.rbegin();

    custom_group_assignment_[worker] = &group;
    custom_group_threads_[group.id] = setupThread(group.id, true, name);

    worker->triggerSwitchThreadRequest(custom_group_threads_[group.id], group.id);

    return group.id;
}

void ThreadPool::deleteThreadGroup(int group_id)
{
    // check if group exists
    std::map<int, QThread*>::iterator pos = custom_group_threads_.find(group_id);
    if(pos == custom_group_threads_.end()) {
        throw std::runtime_error("attempted to delete non-existing thread group");
    }

    // check if group is empty
    for(std::map<NodeWorker*, Group*>::iterator it = custom_group_assignment_.begin();
        it != custom_group_assignment_.end();
        ++it) {
        Group& group = *it->second;
        if(group.id == group_id) {
            throw std::runtime_error("attempted to delete non-empty thread group");
        }
    }

    // delete the thread
    pos->second->quit();
    custom_group_threads_.erase(pos);

    // delete the group
    for(std::vector<Group>::iterator it = custom_groups_.begin();
        it != custom_groups_.end();) {
        Group& group = *it;
        if(group.id == group_id) {
            it = custom_groups_.erase(it);
        } else {
            ++it;
        }
    }

    // check if we can reuse the id easily
    if(group_id == next_id - 1) {
        next_id = group_id;
    }
}

std::string ThreadPool::nextName()
{
    std::stringstream name;
    name << "Thread " << next_id;
    return name.str();
}

void ThreadPool::nodeAdded(NodeWorkerPtr node_worker)
{
    private_thread_[node_worker.get()] = false;

    if(enable_threading_) {
        if(!grouping_) {
            usePrivateThreadFor(node_worker.get());
        } else {
            // if we use grouping -> wait for structure change
            useDefaultThreadFor(node_worker.get());
        }
    } else {
        // this is the one thread to use when threading is disabled
        // we need this to make the ui responsive
        useDefaultThreadFor(node_worker.get());
    }
}

void ThreadPool::structureChanged()
{
    if(enable_threading_ && grouping_) {
        std::set<int> components = assignGroupThreads();
        deleteEmptyGroupThreads(components);
    }
}


void ThreadPool::deleteEmptyGroupThreads(const std::set<int>& components)
{
    for(std::map<int, QThread*>::iterator it = component_group_threads_.begin(); it != component_group_threads_.end();) {
        int component = it->first;
        if(components.find(component) == components.end()) {
            QThread* thread = it->second;
            thread->quit();
            component_group_threads_.erase(it++);
        } else {
            ++it;
        }
    }
}

std::set<int> ThreadPool::assignGroupThreads()
{
    std::set<int> components;
    foreach(NodeWorker* node_worker, graph_->getAllNodeWorkers()) {
        if(custom_group_assignment_.find(node_worker) != custom_group_assignment_.end()) {
            // this worker has a custom group!
            continue;
        }

        if(private_thread_[node_worker]) {
            // this worker has a custom private thread
            continue;
        }

        int component = graph_->getComponent(node_worker->getUUID()) + 1;
        if(!component_group_threads_[component]) {
            std::stringstream name;
            name << "Component " << component;
            component_group_threads_[component] = setupThread(-component, false, name.str().c_str());
        }

        components.insert(component);

        node_worker->triggerSwitchThreadRequest(component_group_threads_[component], UNDEFINED_THREAD);
    }
    return components;
}



QThread * ThreadPool::setupThread(int id, bool custom, const std::string& name)
{
    QThread * thread = new QThread;
    thread->setObjectName(name.c_str());
    thread->setProperty("id", QVariant(id));
    thread->setProperty("custom", custom);
    thread->setProperty("name", QVariant(QString::fromStdString(name)));
    thread->start();
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    return thread;
}
