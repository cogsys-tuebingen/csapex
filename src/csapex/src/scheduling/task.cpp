/// HEADER
#include <csapex/scheduling/task.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;

Task::Task(const std::string& name, std::function<void ()> callback, TaskGenerator *parent)
    : parent_(parent), name_(name), callback_(callback)
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
