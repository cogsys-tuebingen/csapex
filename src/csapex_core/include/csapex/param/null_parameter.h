#ifndef NULL_PARAMETER_H
#define NULL_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT NullParameter : public ParameterImplementation<NullParameter, 0x000>
{
public:
    typedef std::shared_ptr<NullParameter> Ptr;

public:
    NullParameter();
    explicit NullParameter(const std::string& name, const ParameterDescription& description);
    virtual ~NullParameter();


    bool hasState() const override;

    const std::type_info& type() const override;
    std::string toStringImpl() const override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

protected:
    void get_unsafe(boost::any& out) const override;
    bool set_unsafe(const boost::any& v) override;
};


template <>
inline std::string serializationName<NullParameter>()
{
    return "null";
}

}  // namespace param
}  // namespace csapex

#endif  // NULL_PARAMETER_H
