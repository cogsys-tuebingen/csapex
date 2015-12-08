#ifndef STRING_LIST_PARAMETER_H
#define STRING_LIST_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>

namespace csapex {
namespace param {

class StringListParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<StringListParameter> Ptr;

public:
    StringListParameter();
    explicit StringListParameter(const std::string& name, const ParameterDescription &description);
    virtual ~StringListParameter();

    void add(const std::string& value);
    void remove(std::size_t i);
    void removeAll(const std::string& value);

    std::size_t count() const;
    std::vector<std::string> getValues() const;

    virtual int ID() const { return 0x00A; }
    virtual std::string TYPE() const { return "string_list"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

protected:
    virtual boost::any get_unsafe() const override;
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
