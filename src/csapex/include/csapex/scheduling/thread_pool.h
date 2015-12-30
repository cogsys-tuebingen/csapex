#ifndef THREAD_POOL_H
#define THREAD_POOL_H

/// COMPONENT
#include <csapex/scheduling/executor.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/core/exception_handler.h>

/// SYSTEM
#include <map>
#include <vector>
#include <set>
#include <yaml-cpp/yaml.h>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class CsApexCore;

class ThreadPool : public Executor
{
public:
    ThreadPool(csapex::ExceptionHandler &handler, bool enable_threading, bool grouping);

    virtual void performStep() override;

    virtual void stop() override;
    virtual void clear() override;

    std::vector<ThreadGroupPtr> getGroups();
    ThreadGroup* getGroup(int id);
    ThreadGroup* getGroupFor(TaskGenerator* generator);

    std::string nextName();

    virtual void add(TaskGenerator*) override;
    virtual void remove(TaskGenerator *) override;

    void usePrivateThreadFor(TaskGenerator* task);
    void addToGroup(TaskGenerator* task, int group_id);

    int createNewGroupFor(TaskGenerator *task, const std::string &name);

    void useDefaultThreadFor(TaskGenerator *task);

    void saveSettings(YAML::Node&);
    void loadSettings(YAML::Node&);

protected:
    void pauseChanged(bool pause) override;
    void steppingChanged(bool performStep) override;

private:
    void assignGeneratorToGroup(TaskGenerator* task, ThreadGroup* group);

    bool isInPrivateThread(TaskGenerator* task) const;
    bool isInGroup(TaskGenerator* task, int id) const;

    void checkIfStepIsDone();
//    void clearGroup(ThreadGroup *group);

private:
    ExceptionHandler& handler_;

    bool enable_threading_;
    bool grouping_;

    ThreadGroupPtr default_group_;

    std::vector<ThreadGroupPtr> groups_;
    std::map<TaskGenerator*, ThreadGroup*> group_assignment_;
};

}

#endif // THREAD_POOL_H
