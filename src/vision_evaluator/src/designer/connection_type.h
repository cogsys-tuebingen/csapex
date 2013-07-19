#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

class ConnectionType
{
public:
    typedef boost::shared_ptr<ConnectionType> Ptr;
    typedef const boost::shared_ptr<ConnectionType> ConstPtr;

public:
    ConnectionType(const std::string &name);
    virtual ~ConnectionType();

    virtual ConnectionType::Ptr clone() = 0;
    static ConnectionType::Ptr makeDefault();

    virtual bool canConnectTo(ConnectionType::Ptr other_side);

    std::string name();

private:
    std::string name_;

public:
    static ConnectionType::Ptr default_;
};

#endif // CONNECTION_TYPE_H
