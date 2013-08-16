#ifndef CONNECTION_TYPE_H
#define CONNECTION_TYPE_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

namespace csapex {

class ConnectionType
{
public:
    typedef boost::shared_ptr<ConnectionType> Ptr;
    typedef const boost::shared_ptr<ConnectionType> ConstPtr;

public:
    ConnectionType(const std::string &name);
    virtual ~ConnectionType();

    virtual ConnectionType::Ptr clone() = 0;
    virtual ConnectionType::Ptr toType() = 0;
    static ConnectionType::Ptr makeDefault();

    virtual bool canConnectTo(ConnectionType::Ptr other_side);
    virtual bool acceptsConnectionFrom(ConnectionType* other_side);

    std::string name();

    virtual void write(std::ostream& out);

    virtual void writeYaml(YAML::Emitter& yaml) = 0;
    virtual void readYaml(YAML::Node& node) = 0;
private:
    std::string name_;

public:
    static ConnectionType::Ptr default_;
};

}

inline std::ostream& operator << (std::ostream& out, csapex::ConnectionType& obj)
{
    obj.write(out);
    return out;
}

#endif // CONNECTION_TYPE_H
