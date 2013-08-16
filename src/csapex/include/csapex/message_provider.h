#ifndef MESSAGE_PROVIDER_H
#define MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/memento.h>
#include <csapex/message.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QBoxLayout>

namespace csapex
{

class MessageProvider : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<MessageProvider> Ptr;

public:
    static const MessageProvider::Ptr NullPtr;

    ConnectionType::ConstPtr getType() const;

    virtual void load(const std::string& file) = 0;
    virtual void insert(QBoxLayout*) {}

    void setName(const std::string& name);
    std::string getName() const;

    virtual bool hasNext() = 0;
    virtual connection_types::Message::Ptr next() = 0;

    virtual std::vector<std::string> getExtensions() const = 0;

    virtual Memento::Ptr getState() const = 0;
    virtual void setState(Memento::Ptr memento) = 0;

protected:
    void setType(ConnectionType::Ptr type);

private:
    ConnectionType::Ptr type_;

    std::string name_;
};

}

#endif // MESSAGE_PROVIDER_H
