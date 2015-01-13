#ifndef TRIGGER_PARAM_H
#define TRIGGER_PARAM_H

/// COMPONENT
#include <utils_param/parameter.h>

namespace param {


class TriggerParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<TriggerParameter> Ptr;

public:
    TriggerParameter();
    explicit TriggerParameter(const std::string& name, const ParameterDescription& description);
    virtual ~TriggerParameter();

    virtual int ID() const { return 0x007; }
    virtual std::string TYPE() const { return "trigger"; }

    void trigger();
    virtual bool hasState() const;

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
};

}
#endif // TRIGGER_PARAM_H
