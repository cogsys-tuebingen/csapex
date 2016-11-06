#ifndef NODE_MODIFIER_H
#define NODE_MODIFIER_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/multi_connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/uuid.h>


namespace csapex
{
class CSAPEX_EXPORT NodeModifier
{
public:
    NodeModifier();
    virtual ~NodeModifier();

    void setNodeWorker(NodeWorker* worker);

    /*
     * MESSAGES
     */

    /// "real" messages
    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<std::is_base_of<TokenData, T>::value >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<T>(), label);
    }

    /// "container" messages
    template <typename Container, typename T>
    Input* addInput(const std::string& label) {
        Container::template registerType<T>();
        return addInput(Container::template make<T>(), label, false);
    }
    template <typename Container, typename T>
    Input* addOptionalInput(const std::string& label) {
        Container::template registerType<T>();
        return addInput(Container::template make<T>(), label, true);
    }
    template <typename Container, typename T>
    Output* addOutput(const std::string& label) {
        Container::template registerType<T>();
        return addOutput(Container::template make<T>(), label);
    }
    template <typename Container, typename T>
    Event* addEvent(const std::string& label) {
        Container::template registerType<T>();
        return addEvent(Container::template make<T>(), label);
    }
    template <typename Container, typename T>
    Slot* addSlot(const std::string& label, std::function<void(const TokenPtr&)> callback, bool active = false, bool asynchronous = false) {
        Container::template registerType<T>();
        return addSlot(Container::template make<T>(), label, callback, active);
    }



    /// "direct" messages
    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericPointerMessage<T>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, T>::registerConversion();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericPointerMessage<T>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, T>::registerConversion();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericPointerMessage<T>),
                      "connection_types::GenericPointerMessage is not included: "
                      "#include <csapex/msg/generic_pointer_message.hpp>");
        connection_types::MessageConversionHook<connection_types::GenericPointerMessage, T>::registerConversion();
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label);
    }


    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericValueMessage<T>),
                      "connection_types::GenericValueMessage is not included: "
                      "#include <csapex/msg/generic_value_message.hpp>");
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericValueMessage<T>),
                      "connection_types::GenericValueMessage is not included: "
                      "#include <csapex/msg/generic_value_message.hpp>");
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        static_assert(IS_COMPLETE(connection_types::GenericValueMessage<T>),
                      "connection_types::GenericValueMessage is not included: "
                      "#include <csapex/msg/generic_value_message.hpp>");
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label);
    }


    /// multiple input types allowed
    template <typename... Types>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types...>(), label, false);
    }

    template <typename... Types>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types...>(), label, true);
    }

    virtual bool isParameterInput(Input* in) const = 0;
    virtual bool isParameterOutput(Output* in) const = 0;


    /*
     * SIGNALING
     */
    Slot* addActiveSlot(const std::string& label, std::function<void()> callback, bool asynchronous = false);
    Slot* addSlot(const std::string& label, std::function<void()> callback, bool active = false, bool asynchronous = false);

    template <typename T>
    Slot* addTypedSlot(const std::string& label, std::function<void(const TokenPtr&)> callback, bool active = false, bool asynchronous = false)
    {
        return addSlot(connection_types::makeEmptyMessage<T>(), label, callback, active, asynchronous);
    }    
    template <typename Container, typename T>
    Slot* addTypedSlot(const std::string& label, std::function<void(const TokenPtr&)> callback, bool active = false, bool asynchronous = false) {
        Container::template registerType<T>();
        return addSlot(Container::template make<T>(), label, callback, active, asynchronous);
    }

    template <typename T>
    Event* addEvent(const std::string& label)
    {
        return addEvent(connection_types::makeEmptyMessage<T>(), label);
    }
    Event* addEvent(const std::string& label);


    std::vector<InputPtr> getMessageInputs() const;
    std::vector<OutputPtr> getMessageOutputs() const;
    std::vector<SlotPtr> getSlots() const;
    std::vector<EventPtr> getEvents() const;

    virtual void removeInput(const UUID& uuid) = 0;
    virtual void removeOutput(const UUID& uuid) = 0;
    virtual void removeEvent(const UUID& uuid) = 0;
    virtual void removeSlot(const UUID& uuid) = 0;


    /*
     * MISCELLANEOUS
     */

    virtual bool isSource() const = 0;
    virtual void setIsSource(bool source) = 0;

    virtual bool isSink() const = 0;
    virtual void setIsSink(bool sink) = 0;

    bool isProcessingEnabled() const;
    void setProcessingEnabled(bool enabled);

    bool isError() const;
    void setNoError();
    void setError(const std::string& msg);
    void setWarning(const std::string& msg);

    NodeWorker* getNodeWorker() const;

    /**
     * Raw construction, handle with care!
     */
    virtual Input* addInput(TokenDataConstPtr type, const std::string& label, bool optional) = 0;
    virtual Output* addOutput(TokenDataConstPtr type, const std::string& label) = 0;
    virtual Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (Slot*, const TokenPtr& )> callback, bool active, bool asynchronous) = 0;
    virtual Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr& )> callback, bool active, bool asynchronous) = 0;
    virtual Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void ()> callback, bool active, bool asynchronous) = 0;
    virtual Event* addEvent(TokenDataConstPtr type, const std::string& label) = 0;

protected:
    virtual std::vector<ConnectablePtr> getExternalConnectors() const = 0;
    virtual std::vector<InputPtr> getExternalInputs() const = 0;
    virtual std::vector<OutputPtr> getExternalOutputs() const = 0;
    virtual std::vector<SlotPtr> getExternalSlots() const = 0;
    virtual std::vector<EventPtr> getExternalEvents() const = 0;

private:
    mutable NodeWorker* node_worker_;
};

}

#endif // NODE_MODIFIER_H
