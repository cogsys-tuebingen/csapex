#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/shared_ptr.hpp>

namespace csapex {

class ConnectionType
{
public:
    typedef boost::shared_ptr<ConnectionType> Ptr;
    typedef boost::shared_ptr<const ConnectionType> ConstPtr;

public:
    static void setDefaultConnectionType(ConnectionType::Ptr type) {
        default_ = type;
    }

    static ConnectionType::Ptr getDefaultConnectionType();

public:
    ConnectionType(const std::string &name);
    virtual ~ConnectionType();

    template <typename R>
    boost::shared_ptr<R> cloneAs() const
    {
        return boost::dynamic_pointer_cast<R>(clone());
    }

    virtual ConnectionType::Ptr clone() const = 0;
    virtual ConnectionType::Ptr toType() const = 0;
    static ConnectionType::Ptr makeDefault();

    virtual bool isValid() const;

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    virtual std::string name() const;
    std::string rawName() const;

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

    virtual void writeRaw(const std::string& file, const std::string &suffix) const;

protected:
    void setName(const std::string& name);

private:
    std::string name_;
    mutable int seq_no_;

private:
    static ConnectionType::Ptr default_;
};

}

#endif // CONNECTION_TYPE_H
