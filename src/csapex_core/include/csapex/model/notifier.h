#ifndef NOTIFIER_H
#define NOTIFIER_H

/// COMPONENT
#include <csapex/model/notification.h>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

#define NOTIFICATION(args) \
{ Notification n; n.error = ErrorState::ErrorLevel::ERROR; n << args; notification(n); }
#define NOTIFICATION_WARN(args) \
{ Notification n; n.error = ErrorState::ErrorLevel::WARNING; n << args; notification(n); }
#define NOTIFICATION_INFO(args) \
{ Notification n; n.error = ErrorState::ErrorLevel::NONE; n << args; notification(n); }

#define NOTIFICATION_AUUID(auuid, args) \
{ Notification n; n.error = ErrorState::ErrorLevel::ERROR; n << args; n.auuid = auuid; notification(n); }
#define NOTIFICATION_WARN_AUUID(auuid, args) \
{ Notification n; n.error = ErrorState::ErrorLevel::WARNING; n << args; n.auuid = auuid; notification(n); }
#define NOTIFICATION_INFO_AUUID(auuid, args) \
{ Notification n; n.error = ErrorState::ErrorLevel::NONE; n << args; n.auuid = auuid; notification(n); }

class Notifier
{
public:
    virtual ~Notifier();

protected:
    Notifier();

public:
    slim_signal::Signal<void(Notification)> notification;
};

}

#endif // NOTIFIER_H
