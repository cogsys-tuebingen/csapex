#ifndef CTRL_STATE_PUBLISHER_H
#define CTRL_STATE_PUBLISHER_H
#include <computation/cmp_state_publisher.hpp>
#include <QObject>


class QStatePublisher : public QObject, public CMPStatePublisher
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<QStatePublisher> Ptr;

    QStatePublisher();

    void registerListener(QObject *object);

    void removeListener(QObject *object);

Q_SIGNALS:
    void state_update(const boost::any &update);

protected:
    void publish_impl(const boost::any &to_publish);

};

#endif // CTRL_STATE_PUBLISHER_H
