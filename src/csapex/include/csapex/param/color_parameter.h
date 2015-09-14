#ifndef COLOR_PARAMETER_H
#define COLOR_PARAMETER_H

/// COMPONENT
#include <csapex/param/value_parameter.h>

namespace param {


class ColorParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<ColorParameter> Ptr;

public:
    ColorParameter();
    explicit ColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);

    virtual const std::type_info &type() const;

    virtual int ID() const { return 0x002; }
    virtual std::string TYPE() const { return "color"; }

    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);
    void set(const std::vector<int> &v);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

    std::vector<int> def() const;
    std::vector<int> value() const;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & colors_;
    }

protected:
    virtual boost::any get_unsafe() const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    std::vector<int> colors_;
    std::vector<int> def_;
};

}

#endif // COLOR_PARAMETER_H
