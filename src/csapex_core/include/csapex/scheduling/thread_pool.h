#ifndef THREAD_POOL_H
#define THREAD_POOL_H

/// COMPONENT
#include <csapex/scheduling/executor.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/core/exception_handler.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/profiling/profilable.h>
#include <csapex/utility/yaml.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <map>
#include <vector>
#include <set>

namespace csapex
{
class CsApexCore;

class CSAPEX_CORE_EXPORT ThreadPool : public Executor, public Observer, public Profilable
{
public:
    ThreadPool(csapex::ExceptionHandler& handler, bool enable_threading, bool grouping, bool initially_paused);
    ThreadPool(Executor* parent, csapex::ExceptionHandler& handler, bool enable_threading, bool grouping, bool initially_paused);
    ~ThreadPool() override;

    bool isThreadingEnabled() const;
    bool isGroupingEnabled() const;

    void performStep() override;

    void start() override;
    void stop() override;
    void clear() override;

    bool isRunning() const override;

    std::size_t getGroupCount() const;
    ThreadGroup* getGroupAt(std::size_t pos);

    std::vector<ThreadGroupPtr> getGroups();
    ThreadGroup* getDefaultGroup();
    ThreadGroup* getGroup(int id);
    ThreadGroup* getGroupFor(TaskGenerator* generator);

    std::string nextName();

    void add(TaskGenerator*) override;
    void remove(TaskGenerator*) override;

    void usePrivateThreadFor(TaskGenerator* task);
    void addToGroup(TaskGenerator* task, int group_id);

    ThreadGroup* createGroup(const std::string& name, int id = -1);
    int createNewGroupFor(TaskGenerator* task, const std::string& name);
    void removeGroup(int id);

    void useDefaultThreadFor(TaskGenerator* task);

    void saveSettings(YAML::Node&);
    void loadSettings(YAML::Node&);

    void setPrivateThreadGroupCpuAffinity(const std::vector<bool>& affinity);
    std::vector<bool> getPrivateThreadGroupCpuAffinity() const;

    void setSuppressExceptions(bool suppress_exceptions) override;

    void useProfiler(std::shared_ptr<Profiler> profiler) override;

public:
    slim_signal::Signal<void(ThreadGroupPtr)> group_created;
    slim_signal::Signal<void(ThreadGroupPtr)> group_removed;

private:
    slim_signal::Signal<void()> private_group_cpu_affinity_changed;

protected:
    void pauseChanged(bool pause) override;
    void steppingChanged(bool performStep) override;

    bool isStepDone() override;

    void checkIfStepCanBeDone();

private:
    void setup();
    void assignGeneratorToGroup(TaskGenerator* task, ThreadGroup* group);

    bool isInPrivateThread(TaskGenerator* task) const;
    bool isInGroup(TaskGenerator* task, int id) const;

    //    void clearGroup(ThreadGroup *group);
    void removeGroup(ThreadGroup* group);

private:
    ExceptionHandler& handler_;

    TimedQueuePtr timed_queue_;

    bool enable_threading_;
    bool grouping_;

    ThreadGroupPtr default_group_;

    std::vector<ThreadGroupPtr> groups_;
    std::map<TaskGenerator*, ThreadGroup*> group_assignment_;
    std::map<TaskGenerator*, slim_signal::ScopedConnection> group_connection_;

    CpuAffinityPtr private_group_cpu_affinity_;
    std::map<ThreadGroup*, std::vector<slim_signal::ScopedConnection>> private_group_connections_;

    bool suppress_exceptions_;
};

}  // namespace csapex

#endif  // THREAD_POOL_H
