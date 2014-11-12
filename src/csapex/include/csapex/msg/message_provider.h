#ifndef MESSAGE_PROVIDER_H
#define MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/msg/message.h>
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
    virtual void parameterChanged();

    void setName(const std::string& name);
    std::string getName() const;

    std::size_t slotCount() const;

    virtual bool hasNext() = 0;
    virtual connection_types::Message::Ptr next(std::size_t slot) = 0;

    virtual std::vector<std::string> getExtensions() const = 0;

    virtual Memento::Ptr getState() const = 0;
    virtual void setParameterState(Memento::Ptr memento) = 0;

    std::vector<param::ParameterPtr> getParameters() const;

public:
    boost::signals2::signal<void(std::size_t)> slot_count_changed;

protected:
    MessageProvider();
    void setType(ConnectionType::Ptr type);
    void setSlotCount(std::size_t slot_count);

protected:
    GenericState state;

private:
    ConnectionType::Ptr type_;

    std::string name_;
    std::size_t slot_count_;
};

}

#endif // MESSAGE_PROVIDER_H
