#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

class ConnectionType
{
public:
    typedef boost::shared_ptr<ConnectionType> Ptr;

public:
    ConnectionType();
    virtual ~ConnectionType();

    virtual ConnectionType::Ptr clone() = 0;
};

#endif // CONNECTION_TYPE_H
