#ifndef THREAD_POOL_H
#define THREAD_POOL_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <map>
#include <vector>
#include <set>
#include <yaml-cpp/yaml.h>
#include <QThread>

namespace csapex
{

class CsApexCore;

class ThreadPool
{
public:
    enum {
        UNDEFINED_THREAD = -1,
        PRIVATE_THREAD = 0,
        MINIMUM_THREAD_ID = 1
    };

    struct Group
    {
        Group(int id, std::string name)
            : id(id), name(name)
        {}

        int id;
        std::string name;
    };

public:
    ThreadPool(Graph* graph, bool enable_threading, bool grouping);

    std::vector<Group> getCustomGroups();
    Group getCustomGroup(int id);

    std::string nextName();

    void nodeAdded(NodeWorkerPtr);
    void nodeRemoved(NodeWorkerPtr);
    void structureChanged();

    void usePrivateThreadFor(NodeWorker* worker);
    void switchToThread(NodeWorker* worker, int group_id);

    int createNewThreadGroupFor(NodeWorker* worker, const std::string &name);
    void deleteThreadGroup(int group_id);

    void useDefaultThreadFor(NodeWorker *node_worker);

    void saveSettings(YAML::Node&);
    void loadSettings(YAML::Node&);

private:
    QThread *setupThread(int id, bool custom, const std::string &name);
    std::set<int> assignGroupThreads();
    void deleteEmptyGroupThreads(const std::set<int> &components);
    void deletePrivateThread(NodeWorker *worker);

private:
    int next_id;

    Graph *graph_;
    bool enable_threading_;
    bool grouping_;

    std::map<int, QThread*> component_group_threads_;

    std::vector<Group> custom_groups_;
    std::map<NodeWorker*, Group*> custom_group_assignment_;
    std::map<int, QThread*> custom_group_threads_;

    std::map<NodeWorker*, QThread*> private_threads_;
    std::map<NodeWorker*, bool> private_thread_;
};

}

#endif // THREAD_POOL_H
