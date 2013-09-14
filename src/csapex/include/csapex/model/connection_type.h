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
    typedef boost::shared_ptr<const ConnectionType> ConstPtr;

public:
    template <typename C>
    static void setDefaultConnectionType() {
        default_ = C::make();
    }

    static ConnectionType::Ptr getDefaultConnectionType();

public:
    ConnectionType(const std::string &name);
    virtual ~ConnectionType();

    virtual ConnectionType::Ptr clone() = 0;
    virtual ConnectionType::Ptr toType() = 0;
    static ConnectionType::Ptr makeDefault();

    virtual bool canConnectTo(ConnectionType::ConstPtr other_side, bool move) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    std::string name() const;

    virtual void write(std::ostream& out);

    virtual void writeYaml(YAML::Emitter& yaml) = 0;
    virtual void readYaml(YAML::Node& node) = 0;
private:
    std::string name_;

private:
    static ConnectionType::Ptr default_;
};

}

inline std::ostream& operator << (std::ostream& out, csapex::ConnectionType& obj)
{
    obj.write(out);
    return out;
}

#endif // CONNECTION_TYPE_H
