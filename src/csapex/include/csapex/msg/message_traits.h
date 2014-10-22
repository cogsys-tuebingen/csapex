#ifndef MESSAGE_TRAITS_H
#define MESSAGE_TRAITS_H

/// COMPONENT
#include <csapex/model/connection_type.h>
#include <csapex/utility/tmp.hpp>

/// SYSTEM
#include <string>
#include <boost/type_traits/remove_reference.hpp>

namespace csapex {
namespace connection_types {

/// TRAITS
template <typename T>
struct type;

template <typename T>
std::string name()
{
    typedef typename boost::remove_const<T>::type TT;
    return type<TT>::name();
}

template <typename T>
boost::shared_ptr<T> makeEmpty()
{
    return boost::shared_ptr<T>(new T);
}

template <typename T>
boost::shared_ptr<T> makeEmptyMessage(
        typename boost::disable_if<boost::is_const<T> >::type* = 0)
{
    return makeEmpty<T>();
}
template <typename T>
boost::shared_ptr<typename boost::remove_const<T>::type > makeEmptyMessage(
        typename boost::enable_if<boost::is_const<T> >::type* = 0)
{
    typedef typename boost::remove_const<T>::type TT;
    return makeEmpty<TT>();
}


HAS_MEM_TYPE(Ptr, has_ptr_member);
HAS_MEM_TYPE(element_type, has_elem_type_member);

template <typename M>
struct should_use_pointer_message {
    static const bool value = boost::type_traits::ice_and<
    boost::is_class<M>::value,
    has_ptr_member<M>::value,
    boost::type_traits::ice_not< boost::is_same<std::string, M>::value >::value,
    boost::type_traits::ice_not< boost::is_base_of<ConnectionType, M>::value >::value
    >::value;
};

template <typename M>
struct should_use_value_message {
    static const bool value = boost::type_traits::ice_and<
    boost::type_traits::ice_not<
            should_use_pointer_message<M>::value
            >::value,
    boost::type_traits::ice_not< has_elem_type_member<M>::value >::value, // reject shared_ptr
    boost::type_traits::ice_not< boost::is_base_of<ConnectionType, M>::value >::value
    >::value;
};

}
}


#endif // MESSAGE_TRAITS_H
