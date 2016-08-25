/// HEADER
#include <csapex/utility/singleton.hpp>

using namespace csapex;

std::vector<SingletonInterface*> SingletonInterface::instances_;
std::mutex SingletonInterface::instances_mutex_;

SingletonInterface::SingletonInterface()
{
    std::unique_lock<std::mutex> lock(instances_mutex_);
    instances_.push_back(this);
}

void SingletonInterface::shutdownAll()
{
    std::unique_lock<std::mutex> lock(instances_mutex_);
    for(SingletonInterface* si : instances_) {
        si->shutdown();
    }
}

void SingletonInterface::shutdown()
{

}
