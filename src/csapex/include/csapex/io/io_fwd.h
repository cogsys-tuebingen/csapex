#ifndef IO_FWD_H
#define IO_FWD_H

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
FWD(Server);
FWD(Session);
FWD(BroadcastMessage);
FWD(RawMessage);
FWD(Request);
FWD(Response);
FWD(Feedback);
}

#undef FWD

#endif // IO_FWD_H

