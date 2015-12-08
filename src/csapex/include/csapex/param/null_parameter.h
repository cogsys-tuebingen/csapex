#ifndef NULL_PARAMETER_H
#define NULL_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>

namespace csapex {
namespace param {

class NullParameter : public Parameter
{
    friend class ParameterFactory;

    typedef std::shared_ptr<NullParameter> Ptr;

public:
    NullParameter();
    explicit NullParameter(const std::string& name, const ParameterDescription &description);
    virtual ~NullParameter();

    virtual int ID() const { return 0x000; }
    virtual std::string TYPE() const { return "null"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

protected:
    virtual boost::any get_unsafe() const override;
    virtual bool set_unsafe(const boost::any& v) override;
};

}
}

#endif // NULL_PARAMETER_H
