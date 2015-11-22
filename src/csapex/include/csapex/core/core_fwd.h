#ifndef CORE_FWD_H
#define CORE_FWD_H

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
FWD(CsApexCore);
FWD(CorePlugin);
FWD(BootstrapPlugin);
FWD(ExceptionHandler);

class Settings;
}

#undef FWD

#endif // CORE_FWD_H

