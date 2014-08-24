#ifndef MESSAGE_TRAITS_H
#define MESSAGE_TRAITS_H

/// COMPONENT
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <string>
#include <boost/type_traits/remove_reference.hpp>

namespace csapex {
namespace connection_types {

/// TRAITS
template <typename T>
struct type;
template <typename T>
struct MessageRegistered;

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


}
}


#endif // MESSAGE_TRAITS_H
