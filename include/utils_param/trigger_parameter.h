#ifndef TRIGGER_PARAM_H
#define TRIGGER_PARAM_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/type_traits.hpp>

namespace param {


class TriggerParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<TriggerParameter> Ptr;

public:
    TriggerParameter();
    explicit TriggerParameter(const std::string& name);
    virtual ~TriggerParameter();

    virtual int ID() const { return 0x007; }
    virtual std::string TYPE() const { return "Trigger"; }

    void trigger();

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doWrite(YAML::Emitter& e) const;
    void doRead(const YAML::Node& n);

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
};

}
#endif // TRIGGER_PARAM_H
