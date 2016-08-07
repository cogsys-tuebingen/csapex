#ifndef token_traits_H
#define token_traits_H

/// COMPONENT
#include <csapex/model/token_data.h>
#include <csapex/utility/tmp.hpp>

/// SYSTEM
#include <string>
#include <type_traits>

namespace csapex {
namespace connection_types {

/// TRAITS
template <typename T>
struct type;

template <typename T>
std::string serializationName()
{
    typedef typename std::remove_const<T>::type TT;
    return type<TT>::name();
}

template <typename T>
std::shared_ptr<T> makeEmpty()
{
    return std::make_shared<T>();
}

template <>
inline std::shared_ptr<TokenData> makeEmpty<TokenData>()
{
    return std::shared_ptr<TokenData>(new TokenData("empty"));
}


template <typename M>
struct MessageContainer
{
    typedef M type;

    static M& access(M& msg) {
        return msg;
    }
    static const M& accessConst(const M& msg) {
        return msg;
    }
};

template <template <class> class Container, typename T, class Enable = void>
class MessageConversionHook
{
public:
    static void registerConversion() {
        // do nothing
    }
};


template <typename T>
std::shared_ptr<T> makeEmptyMessage(
        typename std::enable_if<!std::is_const<T>::value >::type* = 0)
{
    return makeEmpty<T>();
}
template <typename T>
std::shared_ptr<typename std::remove_const<T>::type > makeEmptyMessage(
        typename std::enable_if<std::is_const<T>::value >::type* = 0)
{
    typedef typename std::remove_const<T>::type TT;
    return makeEmpty<TT>();
}


HAS_MEM_TYPE(Ptr, has_ptr_member);
HAS_MEM_TYPE(element_type, has_elem_type_member);

template <typename M>
struct should_use_pointer_message {
    static constexpr bool value =
            std::is_class<M>::value &&
            has_ptr_member<M>::value &&
            !std::is_same<std::string, M>::value &&
            !std::is_base_of<TokenData, M>::value;
};

template <typename M>
struct should_use_value_message {
    static constexpr bool value =
            !should_use_pointer_message<M>::value &&
            !has_elem_type_member<M>::value && // reject shared_ptr
            !std::is_base_of<TokenData, M>::value;
};

}
}


#endif // token_traits_H
