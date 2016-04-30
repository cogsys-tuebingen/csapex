/// HEADER
#include <csapex/scheduling/task.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;

Task::Task(const std::string& name, std::function<void ()> callback, long priority, TaskGenerator *parent)
    : parent_(parent), name_(name), callback_(callback), priority_(priority), scheduled_(false)
{

}

Task::~Task()
{
}

void Task::execute()
{
    callback_();
}

TaskGenerator* Task::getParent() const
{
    return parent_;
}

std::string Task::getName() const
{
    return name_;
}

void Task::setPriority(long priority)
{
    priority_ = priority;
}

long Task::getPriority() const
{
    return priority_;
}

bool Task::isScheduled() const
{
    return scheduled_;
}

void Task::setScheduled(bool scheduled)
{
    scheduled_ = scheduled;
}
