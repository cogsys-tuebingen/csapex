#ifndef TRIGGER_PARAM_H
#define TRIGGER_PARAM_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT TriggerParameter : public ParameterImplementation<TriggerParameter, 0x007>
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<TriggerParameter> Ptr;

public:
    slim_signal::Signal<void(Parameter*)> first_connect;
    slim_signal::Signal<void(Parameter*)> last_disconnect;

public:
    TriggerParameter();
    explicit TriggerParameter(const std::string& name, const ParameterDescription& description);
    virtual ~TriggerParameter();

    virtual std::string TYPE() const override { return "trigger"; }

    void trigger();
    virtual bool hasState() const override;

    virtual const std::type_info &type() const override;
    virtual std::string toStringImpl() const override;

    void doSetValueFrom(const Parameter& other) override;
    void doClone(const Parameter& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
};

}
}
#endif // TRIGGER_PARAM_H
