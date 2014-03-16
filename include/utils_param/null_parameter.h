#ifndef NULL_PARAMETER_H
#define NULL_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

namespace param {

class NullParameter : public Parameter
{
    friend class ParameterFactory;

    typedef boost::shared_ptr<NullParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const NullParameter& p) {
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const NullParameter::Ptr& p) {
        return e;
    }


public:
    NullParameter();
    explicit NullParameter(const std::string& name);
    virtual ~NullParameter();

    virtual int ID() const { return 0x000; }
    virtual std::string TYPE() const { return "Null"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);
};

}

#endif // NULL_PARAMETER_H
