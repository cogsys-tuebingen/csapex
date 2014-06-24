#ifndef MULTI_CONNECTION_TYPE_H
#define MULTI_CONNECTION_TYPE_H

/// COMPONENT
#include <csapex/model/connection_type.h>

/// SYSTE;
#include <boost/mpl/begin_end.hpp>
#include <boost/mpl/next_prior.hpp>
#include <boost/mpl/vector.hpp>

namespace csapex
{

class MultiConnectionType : public ConnectionType
{
public:
    typedef boost::shared_ptr<MultiConnectionType> Ptr;

public:
    MultiConnectionType(const std::vector<ConnectionType::Ptr>& types);

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

public:
    virtual ConnectionType::Ptr clone() ;
    virtual ConnectionType::Ptr toType();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

private:
    std::vector<ConnectionType::Ptr> types_;
};


namespace multi_type {
namespace detail {

template < typename Begin, typename End, typename F, typename A>
struct static_for_each
{
    static void call(A& type)
    {
        typedef typename Begin::type currentType;

        F::template call< currentType >(type);
        static_for_each< typename boost::mpl::next< Begin >::type, End, F, A >::call(type);
    }
};


template < typename End, typename F, typename A >
struct static_for_each< End, End, F, A >
{
    static void call(A& type)
    {
    }
};



template < typename Sequence, typename F, typename A >
void for_each(A& type)
{
    typedef typename boost::mpl::begin< Sequence >::type begin;
    typedef typename boost::mpl::end< Sequence >::type   end;

    static_for_each< begin, end, F, A >::call(type);
}
} // namespace detail



struct AddType
{
    template < typename T >
    static void call(std::vector<ConnectionType::Ptr>& type)
    {
        type.push_back(ConnectionType::Ptr(new T));
    }
};

template <typename Types>
static ConnectionType::Ptr make()
{
    std::vector<ConnectionType::Ptr> types;
    detail::for_each< Types, AddType, std::vector<ConnectionType::Ptr> >(types);
    return MultiConnectionType::Ptr(new MultiConnectionType(types));
}
}
}


#endif // MULTI_CONNECTION_TYPE_H
