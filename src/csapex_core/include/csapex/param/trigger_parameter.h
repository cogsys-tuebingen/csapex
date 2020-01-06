#ifndef TRIGGER_PARAM_H
#define TRIGGER_PARAM_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_core/csapex_param_export.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT TriggerParameter : public ParameterImplementation<TriggerParameter>
{
public:
    typedef std::shared_ptr<TriggerParameter> Ptr;

public:
    slim_signal::Signal<void(Parameter*)> first_connect;
    slim_signal::Signal<void(Parameter*)> last_disconnect;

public:
    TriggerParameter();
    explicit TriggerParameter(const std::string& name, const ParameterDescription& description);
    virtual ~TriggerParameter();

    TriggerParameter& operator=(const TriggerParameter& p);

    void trigger();
    bool hasState() const override;

    const std::type_info& type() const override;
    std::string toStringImpl() const override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

protected:
    void get_unsafe(std::any& out) const override;
    bool set_unsafe(const std::any& v) override;

private:
};

template <>
inline std::string serializationName<TriggerParameter>()
{
    return "trigger";
}

}  // namespace param
}  // namespace csapex
#endif  // TRIGGER_PARAM_H
