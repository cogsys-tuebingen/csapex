#ifndef MSG_FWD_H
#define MSG_FWD_H

/// shared_ptr
#include <memory>

#define FWD(name)                                                                                                                                                                                      \
    class name;                                                                                                                                                                                        \
    typedef std::shared_ptr<name> name##Ptr;                                                                                                                                                           \
    typedef std::unique_ptr<name> name##UniquePtr;                                                                                                                                                     \
    typedef std::weak_ptr<name> name##WeakPtr;                                                                                                                                                         \
    typedef std::shared_ptr<const name> name##ConstPtr;

namespace csapex
{
FWD(Input)
FWD(Output)
FWD(Event)
FWD(Slot)
FWD(InputTransition)
FWD(OutputTransition)
FWD(MessageProvider)
FWD(MessageRenderer)
FWD(MessageAllocator)

namespace connection_types
{
FWD(Message)
FWD(MarkerMessage)
template <typename Type>
struct GenericPointerMessage;
template <typename Type>
struct GenericValueMessage;
FWD(GenericVectorMessage)
}  // namespace connection_types
}  // namespace csapex

/// this is used for generating more readable warnings
namespace warning
{
template <typename T, int>
constexpr auto is_complete(int) -> decltype(sizeof(T), bool{})
{
    return true;
}

template <typename T, int>
constexpr auto is_complete(...) -> bool
{
    return false;
}
}  // namespace warning

#define IS_COMPLETE(T) warning::is_complete<T, __LINE__>(0)

#undef FWD

#endif  // MSG_FWD_H
