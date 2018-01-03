/// HEADER
#include <csapex/view/utility/qobserver.h>

using namespace csapex;

QObserver::QObserver()
{
    QObject::connect(this, &QObserver::handleObservationsRequest, this, &QObserver::handleObservations, Qt::QueuedConnection);
}

QObserver::~QObserver()
{
    stopObserving();

    std::unique_lock<std::recursive_mutex> lock(observation_queue_mutex_);
    observation_queue_.clear();
}

/// MOC
#include "../../../include/csapex/view/utility/moc_qobserver.cpp"
