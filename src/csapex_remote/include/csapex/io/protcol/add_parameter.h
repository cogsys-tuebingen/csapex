#ifndef ADD_PARAMETER_H
#define ADD_PARAMETER_H

/// PROJECT
#include <csapex/io/request_impl.hpp>
#include <csapex/io/response_impl.hpp>
#include <csapex/param/parameter.h>
#include <csapex/serialization/serialization_fwd.h>

namespace csapex
{

class AddParameter
{
public:
    class ParameterRequest : public RequestImplementation<ParameterRequest>
    {
    public:
        ParameterRequest(const AUUID& id, const std::string &name, const std::string &description, boost::any value, bool persistent);
        ParameterRequest(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const override;

        std::string getType() const override
        {
            return "AddParameter";
        }

    private:
        AUUID id_;
        std::string name_;
        std::string description_;
        boost::any value_;
        bool persistent_;
    };


    class ParameterResponse : public ResponseImplementation<ParameterResponse>
    {
    public:
        ParameterResponse(const param::ParameterConstPtr &parameter, uint8_t request_id);
        ParameterResponse(uint8_t request_id);

        virtual void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
        virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

        param::ParameterConstPtr getParameter() const;

        std::string getType() const override
        {
            return "AddParameter";
        }


    private:
        param::ParameterConstPtr param_;
    };


public:
    using RequestT = ParameterRequest;
    using ResponseT = ParameterResponse;
};

}

#endif // ADD_PARAMETER_H
