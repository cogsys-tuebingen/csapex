#ifndef ANGLE_PARAMETER_H
#define ANGLE_PARAMETER_H

/// COMPONENT
#include <csapex/param/value_parameter.h>
#include <csapex/csapex_param_export.h>

/// SYSTEM
#define _USE_MATH_DEFINES
#include <math.h>

namespace csapex {
namespace param {


class CSAPEX_PARAM_EXPORT AngleParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<AngleParameter> Ptr;

public:
    AngleParameter();
    explicit AngleParameter(const std::string& name, const ParameterDescription& description, double angle, double min = -M_PI, double max = M_PI);

    virtual const std::type_info &type() const override;

    virtual int ID() const override { return 0x00B; }
    virtual std::string TYPE() const override { return "angle"; }

    virtual std::string toStringImpl() const override;

    void doSetValueFrom(const Parameter& other) override;
    void doClone(const Parameter& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

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

}
}

#endif // ANGLE_PARAMETER_H

