#include "ctrl_state_publisher.h"
#include <QMetaType>

QStatePublisher::QStatePublisher()
{
    qRegisterMetaType<boost::any>("boost::any");
}

void QStatePublisher::registerListener(QObject *object)
{
    QObject::connect(this, SIGNAL(state_update(boost::any)), object, SLOT(stateUpdate(boost::any)), Qt::QueuedConnection);
}

void QStatePublisher::removeListener(QObject *object)
{
    QObject::disconnect(SIGNAL(state_update(boost::any)), object, SLOT(stateUpdate(boost::any)));
}

void QStatePublisher::publish_impl(const boost::any &to_publish)
{
    Q_EMIT state_update(to_publish);
}
