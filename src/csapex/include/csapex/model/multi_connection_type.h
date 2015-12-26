#ifndef MULTI_CONNECTION_TYPE_H
#define MULTI_CONNECTION_TYPE_H

/// COMPONENT
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <vector>

namespace csapex
{

class MultiConnectionType : public ConnectionType
{
public:
    typedef std::shared_ptr<MultiConnectionType> Ptr;

public:
    MultiConnectionType(const std::vector<ConnectionType::Ptr>& types);

    virtual bool canConnectTo(const ConnectionType* other_side) const override;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const override;

public:
    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

private:
    std::vector<ConnectionType::Ptr> types_;
};


namespace multi_type {
namespace detail {

template <typename... Ts>
struct AddType;

template < typename T, typename... Ts >
struct AddType<T, Ts...>
{
    static void call(std::vector<ConnectionType::Ptr>& types)
    {
        types.push_back(ConnectionType::Ptr(new T));
        AddType<Ts...>::call(types);
    }
};
template <>
struct AddType<>
{
    static void call(std::vector<ConnectionType::Ptr>& /*types*/)
    {
    }
};

} // namespace detail



template <typename... Types>
static ConnectionType::Ptr make()
{
    std::vector<ConnectionType::Ptr> types;
    detail::AddType<Types...>::call(types);
    return MultiConnectionType::Ptr(new MultiConnectionType(types));
}
}
}


#endif // MULTI_CONNECTION_TYPE_H
