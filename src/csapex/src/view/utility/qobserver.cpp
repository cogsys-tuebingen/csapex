/// HEADER
#include <csapex/view/utility/qobserver.h>

using namespace csapex;

QObserver::QObserver()
{
    QObject::connect(this, &QObserver::handleObservationsRequest, this, &QObserver::handleObservations, Qt::QueuedConnection);
}

/// MOC
#include "../../../include/csapex/view/utility/moc_qobserver.cpp"
