#ifndef NODE_MODIFIER_H
#define NODE_MODIFIER_H

/// COMPONENT
#include <csapex/model/multi_connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_factory.h>
#include <csapex/msg/message_traits.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/mpl/vector.hpp>
#include <boost/type_traits.hpp>

namespace csapex
{
class NodeModifier
{
public:
    NodeModifier(NodeWorker* node);

    /*
     * MESSAGES
     */

    /// "real" messages
    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<std::is_base_of<ConnectionType, T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<std::is_base_of<ConnectionType, T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<std::is_base_of<ConnectionType, T>::value >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<T>(), label, false);
    }
    template <typename T>
    Output* addDynamicOutput(const std::string& label,
                      typename std::enable_if<std::is_base_of<ConnectionType, T>::value >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<T>(), label, true);
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
        return addOutput(Container::template make<T>(), label, false);
    }
    template <typename Container, typename T>
    Output* addDynamicOutput(const std::string& label) {
        Container::template registerType<T>();
        return addOutput(Container::template make<T>(), label, true);
    }



    /// "direct" messages
    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, false);
    }
    template <typename T>
    Output* addDynamicOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_pointer_message<T>::value >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, true);
    }


    template <typename T>
    Input* addInput(const std::string& label,
                    typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, false);
    }
    template <typename T>
    Output* addDynamicOutput(const std::string& label,
                      typename std::enable_if<connection_types::should_use_value_message<T>::value >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, true);
    }


    /// multiple input types allowed
    template <typename Types>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types>(), label, false);
    }
    template <typename A, typename B>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B> >(), label, false);
    }
    template <typename A, typename B, typename C>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C> >(), label, false);
    }
    template <typename A, typename B, typename C, typename D>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D> >(), label, false);
    }
    template <typename A, typename B, typename C, typename D, typename E>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D,E> >(), label, false);
    }

    template <typename Types>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types>(), label, true);
    }
    template <typename A, typename B>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B> >(), label, true);
    }
    template <typename A, typename B, typename C>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C> >(), label, true);
    }
    template <typename A, typename B, typename C, typename D>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D> >(), label, true);
    }
    template <typename A, typename B, typename C, typename D, typename E>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D,E> >(), label, true);
    }


    /*
     * SIGNALING
     */
    Slot* addActiveSlot(const std::string& label, std::function<void()> callback);
    Slot* addSlot(const std::string& label, std::function<void()> callback);
    Trigger* addTrigger(const std::string& label);


    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;
    std::vector<Slot*> getSlots() const;
    std::vector<Trigger*> getTriggers() const;

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);
    void removeTrigger(const UUID& uuid);
    void removeSlot(const UUID& uuid);


    /*
     * MISCELLANEOUS
     */

    void setTickEnabled(bool tick);
    void setTickFrequency(double f);

    bool isSource() const;
    void setIsSource(bool source);

    bool isSink() const;
    void setIsSink(bool sink);

    bool isError() const;
    void setNoError();
    void setError(const std::string& msg);
    void setWarning(const std::string& msg);

private:
    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    Output* addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic);

private:
    NodeWorker* node_worker_;
};

}

#endif // NODE_MODIFIER_H
