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
#include <csapex/model/connector_remote_accessors.hpp>
#undef HANDLE_ACCESSOR
        IsConnectedTo,
    };

    class ConnectorRequest : public RequestImplementation<ConnectorRequest>
    {
    public:
        ConnectorRequest(uint8_t request_id);
        ConnectorRequest(ConnectorRequestType request_type, const AUUID& uuid);
        ConnectorRequest(ConnectorRequestType request_type, const AUUID& uuid, const boost::any& payload);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

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


    class NodeResponse : public ResponseImplementation<NodeResponse>
    {
    public:
        NodeResponse(uint8_t request_id);
        NodeResponse(ConnectorRequestType request_type, uint8_t request_id, const AUUID& uuid);
        NodeResponse(ConnectorRequestType request_type, boost::any result, uint8_t request_id, const AUUID& uuid);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

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
    using ResponseT = NodeResponse;
};

}
#endif // CONNECTOR_REQUESTS_H
