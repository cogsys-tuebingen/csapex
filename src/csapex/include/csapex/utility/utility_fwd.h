#ifndef UTILITY_FWD_H
#define UTILITY_FWD_H

/// shared_ptr
#include <memory>

#define FWD(name) \
    class name;\
    typedef std::shared_ptr<name> name##Ptr;\
    typedef std::unique_ptr<name> name##UniquePtr;\
    typedef std::weak_ptr<name> name##WeakPtr;\
    typedef std::shared_ptr<const name> name##ConstPtr;


namespace csapex
{
FWD(Buffer);
FWD(Timer);
FWD(Timable);
FWD(UUID);

class Notification;

class StreamRelay;
}

#undef FWD

#endif // UTILITY_FWD_H

