#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

/// SYSTEM
#include <boost/signals2.hpp>
#include <memory>

namespace csapex {

class ConnectionType
{
public:
    typedef std::shared_ptr<ConnectionType> Ptr;
    typedef std::shared_ptr<const ConnectionType> ConstPtr;

public:
    static void setDefaultConnectionType(ConnectionType::Ptr type) {
        default_ = type;
    }

    static ConnectionType::Ptr getDefaultConnectionType();

public:
    ConnectionType(const std::string &name);
    virtual ~ConnectionType();

    template <typename R>
    std::shared_ptr<R> cloneAs() const
    {
        return std::dynamic_pointer_cast<R>(clone());
    }

    virtual ConnectionType::Ptr clone() const = 0;
    virtual ConnectionType::Ptr toType() const = 0;
    static ConnectionType::Ptr makeDefault();

    virtual bool isValid() const;

    virtual bool isContainer() const;
    virtual Ptr nestedType() const;
    virtual ConstPtr nestedValue(std::size_t i) const;
    virtual void addNestedValue(const ConstPtr& msg);
    virtual std::size_t nestedValueCount() const;

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    virtual std::string name() const;
    std::string rawName() const;

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

    virtual void writeRaw(const std::string& file,  const std::string &base, const std::string &suffix) const;

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
