#ifndef MESSAGE_PROVIDER_H
#define MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/model/message.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QBoxLayout>

namespace csapex
{

class MessageProvider
{
public:
    typedef boost::shared_ptr<MessageProvider> Ptr;

public:
    virtual ~MessageProvider();

    ConnectionType::ConstPtr getType() const;

    virtual void load(const std::string& file) = 0;

    void setName(const std::string& name);
    std::string getName() const;

    virtual bool hasNext() = 0;
    virtual connection_types::Message::Ptr next() = 0;

    virtual std::vector<std::string> getExtensions() const = 0;

    virtual Memento::Ptr getState() const = 0;
    virtual void setState(Memento::Ptr memento) = 0;

    std::vector<param::Parameter::Ptr> getParameters() const;

protected:
    void setType(ConnectionType::Ptr type);

protected:
    GenericState state;

private:
    ConnectionType::Ptr type_;

    std::string name_;

};

}

#endif // MESSAGE_PROVIDER_H
