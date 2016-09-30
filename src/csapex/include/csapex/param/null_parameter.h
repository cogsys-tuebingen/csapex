#ifndef NULL_PARAMETER_H
#define NULL_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT NullParameter : public Parameter
{
    friend class ParameterFactory;

    typedef std::shared_ptr<NullParameter> Ptr;

public:
    NullParameter();
    explicit NullParameter(const std::string& name, const ParameterDescription &description);
    virtual ~NullParameter();

    virtual int ID() const override { return 0x000; }
    virtual std::string TYPE() const override { return "null"; }

    virtual const std::type_info &type() const override;
    virtual std::string toStringImpl() const override;

    void doSetValueFrom(const Parameter& other) override;
    void doClone(const Parameter& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;
};

}
}

#endif // NULL_PARAMETER_H
