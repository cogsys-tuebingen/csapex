#ifndef BOOST_IO_H
#define BOOST_IO_H

/// COMPONENT
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <boost/make_shared.hpp>

namespace csapex
{
// SHARED POINTER (of non-serializable type)
template <typename T>
SerializationBuffer& operator << (SerializationBuffer& data, const boost::shared_ptr<T>& s)
{
    if(s) {
        data << true;
        data << *s;
    } else {
        data << false;
    }
    return data;
}


template <typename T>
const SerializationBuffer& operator >> (const SerializationBuffer& data, boost::shared_ptr<T>& s)
{
    bool valid;
    data >> valid;
    if(!valid) {
        return data;
    }

    // in case T is const, we need to strip that, otherwise we cannot deserialize
    using TT = typename std::remove_const<T>::type;
    boost::shared_ptr<TT> res = boost::make_shared<TT>();
    data >> *res;
    s = res;
    return data;
}
}

#endif // BOOST_IO_H
