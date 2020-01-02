#ifndef CONNECTOR_REQUESTS_H
#define CONNECTOR_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{
class ConnectorRequests
{
public:
    enum class ConnectorRequestType
    {
#define HANDLE_ACCESSOR(_enum, type, function) _enum,
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)
#include <csapex/model/connector_proxy_accessors.hpp>

        IsConnectedTo,
        IsActivelyConnectedTo,
    };

    class ConnectorRequest : public RequestImplementation<ConnectorRequest>
    {
    public:
        ConnectorRequest(uint8_t request_id);
        ConnectorRequest(ConnectorRequestType request_type, const AUUID& uuid);
        ConnectorRequest(ConnectorRequestType request_type, const AUUID& uuid, const boost::any& payload);

        void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "ConnectorRequests";
        }

        boost::any getPayload() const
        {
            return payload_;
        }

    private:
        ConnectorRequestType request_type_;
        AUUID uuid_;
        boost::any payload_;
    };

    class ConnectorResponse : public ResponseImplementation<ConnectorResponse>
    {
    public:
        ConnectorResponse(uint8_t request_id);
        ConnectorResponse(ConnectorRequestType request_type, uint8_t request_id, const AUUID& uuid);
        ConnectorResponse(ConnectorRequestType request_type, boost::any result, uint8_t request_id, const AUUID& uuid);

        void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
        void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        std::string getType() const override
        {
            return "ConnectorRequests";
        }

        template <typename R>
        R getResult() const
        {
            return boost::any_cast<R>(result_);
        }

    private:
        ConnectorRequestType request_type_;
        AUUID uuid_;

        boost::any result_;
    };

public:
    using RequestT = ConnectorRequest;
    using ResponseT = ConnectorResponse;
};

}  // namespace csapex
#endif  // CONNECTOR_REQUESTS_H
