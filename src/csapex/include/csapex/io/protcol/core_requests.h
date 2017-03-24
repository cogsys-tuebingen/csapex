#ifndef CORE_REQUESTS_H
#define CORE_REQUESTS_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class CoreRequests
{
public:
    enum class CoreRequestTarget
    {
        Settings,
        Core,
        Other
    };

    enum class CoreRequestType
    {
        Save,
        Load
    };

    class CoreRequest : public RequestImplementation<CoreRequest>
    {
    public:
        CoreRequest(CoreRequestTarget request_target, CoreRequestType request_type);
        CoreRequest(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        virtual ResponsePtr execute(CsApexCore& core) const override;

        std::string getType() const override
        {
            return "CoreRequests";
        }

    private:
        ResponsePtr executeCore(CsApexCore& core) const;
        ResponsePtr executeSettings(CsApexCore& core) const;

    private:
        CoreRequestTarget request_target_;
        CoreRequestType request_type_;
    };


    class CoreResponse : public ResponseImplementation<CoreResponse>
    {
    public:
        CoreResponse(CoreRequestTarget request_target, CoreRequestType request_type, uint8_t request_id);
        CoreResponse(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data) const override;
        virtual void deserialize(SerializationBuffer& data) override;

        std::string getType() const override
        {
            return "CoreRequests";
        }

    private:
        CoreRequestTarget request_target_;
        CoreRequestType request_type_;
    };


public:
    using RequestT = CoreRequest;
    using ResponseT = CoreResponse;
};

}

#endif // CORE_REQUESTS_H
