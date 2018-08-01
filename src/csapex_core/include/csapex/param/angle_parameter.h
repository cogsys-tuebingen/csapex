#ifndef ANGLE_PARAMETER_H
#define ANGLE_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

/// SYSTEM
#define _USE_MATH_DEFINES
#include <math.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT AngleParameter : public ParameterImplementation<AngleParameter>
{
public:
    typedef std::shared_ptr<AngleParameter> Ptr;

public:
    AngleParameter();
    explicit AngleParameter(const std::string& name, const ParameterDescription& description, double angle, double min = -M_PI, double max = M_PI);

    virtual const std::type_info& type() const override;

    virtual std::string toStringImpl() const override;

    void cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    double min() const;
    double max() const;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    double angle_;
    double min_;
    double max_;
};


template <>
inline std::string serializationName<AngleParameter>()
{
    return "angle";
}

}  // namespace param
}  // namespace csapex

#endif  // ANGLE_PARAMETER_H
