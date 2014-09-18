#ifndef NODE_MODIFIER_H
#define NODE_MODIFIER_H

/// COMPONENT
#include <csapex/model/multi_connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/mpl/vector.hpp>

namespace csapex
{
class NodeModifier
{
public:
    NodeModifier(Node* node);

    /// "real" messages
    template <typename T>
    Input* addInput(const std::string& label,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, false, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<T>(), label, true, false);
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
        return addInput(Container::template make<T>(), label, false, false);
    }
    template <typename Container, typename T>
    Input* addOptionalInput(const std::string& label) {
        Container::template registerType<T>();
        return addInput(Container::template make<T>(), label, true, false);
    }
    template <typename Container, typename T>
    Output* addOutput(const std::string& label) {
        Container::template registerType<T>();
        return addOutput(Container::template make<T>(), label);
    }

    /// "direct" messages
    template <typename T>
    Input* addInput(const std::string& label,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, false, false);
    }
    template <typename T>
    Input* addOptionalInput(const std::string& label,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addInput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label, true, false);
    }
    template <typename T>
    Output* addOutput(const std::string& label,
                            typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* = 0) {
        return addOutput(connection_types::makeEmptyMessage<connection_types::GenericPointerMessage<T> >(), label);
    }


    /// multiple input types allowed
    template <typename Types>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types>(), label, false, false);
    }
    template <typename A, typename B>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B> >(), label, false, false);
    }
    template <typename A, typename B, typename C>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C> >(), label, false, false);
    }
    template <typename A, typename B, typename C, typename D>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D> >(), label, false, false);
    }
    template <typename A, typename B, typename C, typename D, typename E>
    Input* addMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D,E> >(), label, false, false);
    }

    template <typename Types>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make<Types>(), label, true, false);
    }
    template <typename A, typename B>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B> >(), label, true, false);
    }
    template <typename A, typename B, typename C>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C> >(), label, true, false);
    }
    template <typename A, typename B, typename C, typename D>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D> >(), label, true, false);
    }
    template <typename A, typename B, typename C, typename D, typename E>
    Input* addOptionalMultiInput(const std::string& label) {
        return addInput(multi_type::make< boost::mpl::vector<A,B,C,D,E> >(), label, true, false);
    }


private:
    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

private:
    Node* node_;
};

}

#endif // NODE_MODIFIER_H
