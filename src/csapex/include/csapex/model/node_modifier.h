#ifndef NODE_MODIFIER_H
#define NODE_MODIFIER_H

/// COMPONENT
#include <csapex/model/multi_connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_factory.h>
#include <csapex/msg/message_traits.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/csapex_fwd.h>

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
                    typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
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



    /// "direct" messages
    template <typename T>
    Input* addInput(const std::string& label,
                    typename boost::enable_if<connection_types::should_use_pointer_message<T> >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename boost::enable_if<connection_types::should_use_pointer_message<T> >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename boost::enable_if<connection_types::should_use_pointer_message<T> >::type* = 0) {
        MessageFactory::registerDirectMessage<connection_types::GenericPointerMessage, T>();
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label);
    }


    template <typename T>
    Input* addInput(const std::string& label,
                    typename boost::enable_if<connection_types::should_use_value_message<T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                            typename boost::enable_if<connection_types::should_use_value_message<T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label, true);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                      typename boost::enable_if<connection_types::should_use_value_message<T> >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericValueMessage<T> >(), label);
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
    Slot* addSlot(const std::string& label);
    Trigger* addTrigger(const std::string& label);

private:
    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

private:
    NodeWorker* node_worker_;
};

}

#endif // NODE_MODIFIER_H
