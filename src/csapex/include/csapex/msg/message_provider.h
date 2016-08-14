#ifndef MESSAGE_PROVIDER_H
#define MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/msg/message.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <memory>

namespace csapex
{

class CSAPEX_EXPORT MessageProvider
{
public:
    typedef std::shared_ptr<MessageProvider> Ptr;

public:
    virtual ~MessageProvider();

    TokenData::ConstPtr getType() const;

    virtual void load(const std::string& file) = 0;
    virtual void parameterChanged();

    void setName(const std::string& name);
    std::string getName() const;

    std::size_t slotCount() const;
    virtual std::string getLabel(std::size_t slot) const;

    virtual bool hasNext() = 0;
    virtual connection_types::Message::Ptr next(std::size_t slot) = 0;

    virtual void restart();

    virtual std::vector<std::string> getExtensions() const = 0;

    virtual Memento::Ptr getState() const = 0;
    virtual void setParameterState(Memento::Ptr memento) = 0;

    std::vector<csapex::param::ParameterPtr> getParameters() const;

public:
    csapex::slim_signal::Signal<void(std::size_t)> slot_count_changed;


    csapex::slim_signal::Signal<void()> begin;
    csapex::slim_signal::Signal<void()> no_more_messages;

protected:
    MessageProvider();
    void setType(TokenData::Ptr type);
    void setSlotCount(std::size_t slot_count);

protected:
    GenericState state;

private:
    TokenData::Ptr type_;

    std::string name_;
    std::size_t slot_count_;
};

}

#endif // MESSAGE_PROVIDER_H
