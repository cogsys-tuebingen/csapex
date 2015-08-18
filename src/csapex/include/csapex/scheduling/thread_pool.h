#ifndef THREAD_POOL_H
#define THREAD_POOL_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/scheduling/executor.h>

/// SYSTEM
#include <map>
#include <vector>
#include <set>
#include <yaml-cpp/yaml.h>
#include <boost/signals2/signal.hpp>

namespace csapex
{

class CsApexCore;

class ThreadPool : public Executor
{
public:
    ThreadPool(bool enable_threading, bool grouping, bool paused);

    virtual void performStep() override;

    void stop();
    virtual void clear() override;

    std::vector<ThreadGroupPtr> getGroups();
    ThreadGroup* getGroup(int id);
    ThreadGroup* getGroupFor(TaskGenerator* generator);

    std::string nextName();

    void add(TaskGenerator*);
    void remove(TaskGenerator *);

    void usePrivateThreadFor(TaskGenerator* task);
    void addToGroup(TaskGenerator* task, int group_id);

    int createNewGroupFor(TaskGenerator *task, const std::string &name);

    void useDefaultThreadFor(TaskGenerator *task);

    void saveSettings(YAML::Node&);
    void loadSettings(YAML::Node&);

protected:
    void pauseChanged(bool pause);
    void steppingChanged(bool performStep);

private:
    void assignGeneratorToGroup(TaskGenerator* task, ThreadGroup* group);

    bool isInPrivateThread(TaskGenerator* task) const;
    bool isInGroup(TaskGenerator* task, int id) const;

    void checkIfStepIsDone();
//    void clearGroup(ThreadGroup *group);

private:
    bool enable_threading_;
    bool grouping_;

    ThreadGroupPtr default_group_;

    std::vector<ThreadGroupPtr> groups_;
    std::map<TaskGenerator*, ThreadGroup*> group_assignment_;
};

}

#endif // THREAD_POOL_H
