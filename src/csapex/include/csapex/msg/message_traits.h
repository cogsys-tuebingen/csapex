#ifndef MESSAGE_TRAITS_H
#define MESSAGE_TRAITS_H

/// SYSTEM
#include <string>
#include <boost/type_traits/remove_reference.hpp>

namespace csapex {
namespace connection_types {

/// TRAITS
template <typename T>
struct type {};

template <typename T>
std::string name()
{
    typedef typename boost::remove_const<T>::type TT;
    return type<TT>::name();
}

template <typename T>
inline boost::shared_ptr<T> makeEmpty()
{
    return boost::shared_ptr<T>(new T);
}

}
}

#endif // MESSAGE_TRAITS_H
