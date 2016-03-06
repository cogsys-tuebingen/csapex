#ifndef PROXY_HPP
#define PROXY_HPP

/// COMPONENT
#include <csapex/param/param_fwd.h>

namespace csapex
{

namespace param
{

template <class P>
class Proxy : public P
{
public:
    typedef std::shared_ptr<Proxy<P>> Ptr;
    typedef std::shared_ptr<Proxy<P> const> ConstPtr;

public:
    Proxy(const std::shared_ptr<P>& wrapped)
        : P(*wrapped), wrapped_(wrapped)
    {}

private:
    std::shared_ptr<P> wrapped_;
};

}

}

#endif // PROXY_HPP
