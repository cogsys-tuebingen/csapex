#ifndef VALUE_PARAMETER_H
#define VALUE_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT ValueParameter : public ParameterImplementation<ValueParameter>
{
public:
    typedef std::shared_ptr<ValueParameter> Ptr;

public:
    ValueParameter();
    explicit ValueParameter(const std::string& name, const ParameterDescription& description);

    template <typename T>
    ValueParameter(const std::string& name, const ParameterDescription& description, T def)
        : ValueParameter(name, description)
    {
        def_ = def;
    }

    virtual ~ValueParameter();

    virtual const std::type_info& type() const override;
    virtual std::string toStringImpl() const override;

    void cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    template <typename T>
    T def() const
    {
        return read<T>(def_);
    }

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    template <typename T>
    T read(const boost::any& var) const
    {
        try {
            return boost::any_cast<T>(var);

        } catch (const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof ValueParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    boost::any value_;
    boost::any def_;
};


template <>
inline std::string serializationName<ValueParameter>()
{
    return "value";
}

}  // namespace param
}  // namespace csapex

#endif  // VALUE_PARAMETER_H
