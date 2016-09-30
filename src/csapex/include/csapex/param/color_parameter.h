#ifndef COLOR_PARAMETER_H
#define COLOR_PARAMETER_H

/// COMPONENT
#include <csapex/param/value_parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT ColorParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<ColorParameter> Ptr;

public:
    ColorParameter();
    explicit ColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);

    virtual const std::type_info &type() const override;

    virtual int ID() const override { return 0x002; }
    virtual std::string TYPE() const override { return "color"; }

    virtual std::string toStringImpl() const override;

    void doSetValueFrom(const Parameter& other) override;
    void doClone(const Parameter& other) override;
    void set(const std::vector<int> &v);

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    std::vector<int> def() const;
    std::vector<int> value() const;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        ar & colors_;
    }

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    std::vector<int> colors_;
    std::vector<int> def_;
};

}
}

#endif // COLOR_PARAMETER_H
