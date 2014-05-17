#ifndef NODE_MODIFIER_H
#define NODE_MODIFIER_H

/// COMPONENT
#include <csapex/model/multi_connection_type.h>
#include <csapex/model/message.h>
#include <csapex/csapex_fwd.h>

namespace csapex
{

/// FORWARD
template <typename T>
class RosMessageConversionT;

class NodeModifier
{
public:
    NodeModifier(Node* node);

    /// "real" messages
    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addInput(T::make(), label, optional, async);
    }
    template <typename Container, typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addInput(Container::template make<T>(), label, optional, async);
    }

    /// "direct" messages
    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addInput(connection_types::GenericMessage<T>::make(), label, optional, async);
    }
    template <typename Container, typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addInput(Container::template make<T>(), label, optional, async);
    }

    /// "real" messages
    template <typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addOutput(T::make(), label);
    }
    template <typename Container, typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addOutput(Container::template make<T>(), label);
    }

    /// "direct" messages
    template <typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addOutput(connection_types::GenericMessage<T>::make(), label);
    }
    template <typename Container, typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addOutput(Container::template make<T>(), label);
    }


    /// multiple input types allowed
    template <typename Types>
    ConnectorIn* addMultiInput(const std::string& label, bool optional = false, bool async = false) {
        return addInput(multi_type::make<Types>(), label, optional, async);
    }

    // TODO: default for 2, 3, 4, 5; other -> boost mpl vector


private:
    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

private:
    Node* node_;
};

}

#endif // NODE_MODIFIER_H
