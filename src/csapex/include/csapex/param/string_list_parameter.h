#ifndef STRING_LIST_PARAMETER_H
#define STRING_LIST_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT StringListParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<StringListParameter> Ptr;

public:
    StringListParameter();
    explicit StringListParameter(const std::string& name, const ParameterDescription &description);
    virtual ~StringListParameter();

    void add(const std::string& value);
    void setAt(std::size_t i, const std::string& value);
    void remove(std::size_t i);
    void removeAll(const std::string& value);

    std::size_t count() const;
    std::vector<std::string> getValues() const;

    virtual int ID() const override { return 0x00A; }
    virtual std::string TYPE() const override { return "string_list"; }

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
    template <typename T>
    T read(const boost::any& var) const
    {
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof StringListParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::vector<std::string> list_;
};

}
}

#endif // STRING_LIST_PARAMETER_H
