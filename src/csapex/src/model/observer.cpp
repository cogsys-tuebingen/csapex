/// HEADER
#include <csapex/model/observer.h>

using namespace csapex;

Observer::~Observer()
{

}

void Observer::stopObserving()
{
    observed_connections_.clear();
}

void Observer::manageConnection(slim_signal::ScopedConnection&& connection)
{
    observed_connections_.emplace_back(std::move(connection));
}

void Observer::manageConnection(const slim_signal::Connection& connection)
{
    observed_connections_.emplace_back(connection);
}
