#ifndef REQUEST_PARAMETER_H
#define REQUEST_PARAMETER_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

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
        ParameterRequest(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(CsApexCore& core) const override;

        std::string getType() const override
        {
            return "RequestParameter";
        }

    private:
        AUUID id_;
    };


    class ParameterResponse : public ResponseImplementation<ParameterResponse>
    {
    public:
        ParameterResponse(const param::ParameterConstPtr &parameter, uint8_t request_id);
        ParameterResponse(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        param::ParameterConstPtr getParameter() const;

        std::string getType() const override
        {
            return "RequestParameter";
        }


    private:
        param::ParameterConstPtr param_;
    };


};

}

#endif // REQUEST_PARAMETER_H
