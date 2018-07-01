#ifndef SIGNAL_FWD_H
#define SIGNAL_FWD_H

#include <csapex/msg/msg_fwd.h>

#define FWD(name)                                                                                                                                                                                      \
    class name;                                                                                                                                                                                        \
    typedef std::shared_ptr<name> name##Ptr;                                                                                                                                                           \
    typedef std::unique_ptr<name> name##UniquePtr;                                                                                                                                                     \
    typedef std::weak_ptr<name> name##WeakPtr;                                                                                                                                                         \
    typedef std::shared_ptr<const name> name##ConstPtr;

namespace csapex
{
namespace connection_types
{
FWD(Signal)
}

}  // namespace csapex

#undef FWD

#endif  // SIGNAL_FWD_H
