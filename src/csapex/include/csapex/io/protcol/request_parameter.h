#ifndef REQUEST_PARAMETER_H
#define REQUEST_PARAMETER_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{

namespace io
{
    class RequestParameterSerializer;
}

class RequestParameter
{
public:
    class ParameterRequest : public RequestImplementation<ParameterRequest>
    {
    public:
        ParameterRequest(const AUUID& id);
        ParameterRequest();

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(CsApexCore& core) const override;

    private:
        AUUID id_;
    };


    class ParameterResponse : public ResponseImplementation<ParameterResponse>
    {
    public:
        ParameterResponse(const param::ParameterConstPtr &parameter);
        ParameterResponse();

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        param::ParameterConstPtr getParameter() const;

    private:
        param::ParameterConstPtr param_;
    };


};

}

#endif // REQUEST_PARAMETER_H
