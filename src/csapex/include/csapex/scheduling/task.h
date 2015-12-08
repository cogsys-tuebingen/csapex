#ifndef TASK_H
#define TASK_H

/// PROJECT
#include <csapex/scheduling/scheduling_fwd.h>

/// SYSTEM
#include <functional>

namespace csapex
{

class Task
{
public:
    Task(const std::string &name, std::function<void()> callback, TaskGenerator* parent = nullptr);
    virtual ~Task();

    virtual void execute();

    TaskGenerator* getParent() const;
    std::string getName() const;

private:
    TaskGenerator* parent_;
    std::string name_;
    std::function<void()> callback_;
};

}

#endif // TASK_H
