#ifndef TASK_H
#define TASK_H

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <functional>

namespace csapex
{

class CSAPEX_EXPORT Task
{
public:
    Task(const std::string &name, std::function<void()> callback, long priority = 0, TaskGenerator* parent = nullptr);
    virtual ~Task();

    virtual void execute();

    void setPriority(long priority);
    long getPriority() const;

    void setScheduled(bool scheduled);
    bool isScheduled() const;

    TaskGenerator* getParent() const;
    std::string getName() const;

private:
    TaskGenerator* parent_;
    std::string name_;
    std::function<void()> callback_;

    long priority_;
    bool scheduled_;
};

}

#endif // TASK_H
