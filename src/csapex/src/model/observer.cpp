/// HEADER
#include <csapex/model/observer.h>

using namespace csapex;

Observer::~Observer()
{

}

void Observer::stopObserving()
{
    connections_.clear();
}

void Observer::manageConnection(slim_signal::ScopedConnection&& connection)
{
    connections_.emplace_back(std::move(connection));
}

void Observer::manageConnection(slim_signal::Connection&& connection)
{
    connections_.emplace_back(std::move(connection));
}

void Observer::manageConnection(const slim_signal::Connection& connection)
{
    connections_.emplace_back(connection);
}
