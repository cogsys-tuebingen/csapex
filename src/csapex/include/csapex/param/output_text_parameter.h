#ifndef OUTPUT_TEXT_PARAMETER_H
#define OUTPUT_TEXT_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT OutputTextParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<OutputTextParameter> Ptr;

public:
    OutputTextParameter();
    explicit OutputTextParameter(const std::string& name, const ParameterDescription& description);
    virtual ~OutputTextParameter();

    virtual int ID() const override { return 0x00C; }
    virtual std::string TYPE() const override { return "outtext"; }

    virtual const std::type_info &type() const override;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;
    virtual std::string toStringImpl() const override;

    virtual void doSerialize(YAML::Node& n) const override;
    virtual void doDeserialize(const YAML::Node& n) override;

    virtual void doSetValueFrom(const Parameter& other) override;
    virtual void doClone(const Parameter& other) override;

private:
    std::string text_;
};

}
}
#endif // OUTPUT_TEXT_PARAMETER_H
