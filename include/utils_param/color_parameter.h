#ifndef COLOR_PARAMETER_H
#define COLOR_PARAMETER_H

/// COMPONENT
#include <utils_param/value_parameter.h>

namespace param {


class ColorParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<ColorParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const ColorParameter& p) {
        p.doWrite(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const ColorParameter::Ptr& p) {
        p->doWrite(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::ColorParameter& value) {
        value.doRead(node);
    }

    friend void operator >> (const YAML::Node& node, param::ColorParameter::Ptr& value) {
        if(!value) {
            value.reset(new ColorParameter);
        }
        value->doRead(node);
    }

public:
    ColorParameter();
    explicit ColorParameter(const std::string& name, int r, int g, int b); virtual const std::type_info &type() const;

    virtual int ID() const { return 0x002; }
    virtual std::string TYPE() const { return "Color"; }

    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);
    void set(const std::vector<int> &v);

    void doWrite(YAML::Emitter& e) const;
    void doRead(const YAML::Node& n);

    std::vector<int> def() const;
    std::vector<int> value() const;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & colors_;
    }

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    std::vector<int> colors_;
    std::vector<int> def_;
};

}

#endif // COLOR_PARAMETER_H
