#ifndef PARAMETER_SERIALIZATION_HPP
#define PARAMETER_SERIALIZATION_HPP

/// COMPONENT
#include <csapex/param/parameter_serialization.hpp>

namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(Archive & ar, const csapex::param::Parameter * t, unsigned int)
{
    std::string name = t->name();
    ar << name;
}

template<class Archive>
inline void load_construct_data(Archive & ar, csapex::param::Parameter * t, unsigned int)
{
    std::string name;
    ar >> name;
    ::new(t)csapex::param::Parameter(name);
}

template <typename A, typename Key>
inline void serialize(A &ar, typename std::pair<Key, csapex::param::Parameter> &p, const unsigned)
{
    // Do nothing, handled by xxxx_construct_data
}

template <typename A, typename Key>
inline void save_construct_data(A &ar, const std::pair<Key, csapex::param::Parameter> *p, const unsigned v)
{
    using namespace boost::serialization;
    ar << make_nvp("first", p->first);
    save_construct_data(ar, &p->first, v);
    ar << make_nvp("second", p->second);
    save_construct_data(ar, &p->second, v);
}

template <typename A, typename Key>
inline void load_construct_data(A &ar, std::pair<Key, csapex::param::Parameter>* p, const unsigned v)
{
    using namespace boost::serialization;
    typedef BOOST_DEDUCED_TYPENAME
            boost::remove_const<Key>::type typef;
    load_construct_data(ar, &const_cast<typef &>(p->first), v);
    ar >> make_nvp("first", const_cast<typef &>(p->first));

    load_construct_data(ar, &p->second, v);
    ar >> make_nvp("second", p->second);
}

}}
#endif // PARAMETER_SERIALIZATION_HPP
