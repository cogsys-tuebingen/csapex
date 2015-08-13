#ifndef ANGLE_PARAMETER_H
#define ANGLE_PARAMETER_H

/// COMPONENT
#include <utils_param/value_parameter.h>

namespace param {


class AngleParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<AngleParameter> Ptr;

public:
    AngleParameter();
    explicit AngleParameter(const std::string& name, const ParameterDescription& description, double angle, double min = -M_PI, double max = M_PI);

    virtual const std::type_info &type() const;

    virtual int ID() const { return 0x00B; }
    virtual std::string TYPE() const { return "angle"; }

    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

    double min() const;
    double max() const;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & angle_;
    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    double angle_;
    double min_;
    double max_;
};

}

#endif // ANGLE_PARAMETER_H

