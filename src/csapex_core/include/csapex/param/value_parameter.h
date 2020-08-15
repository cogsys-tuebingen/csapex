#ifndef VALUE_PARAMETER_H
#define VALUE_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_core/csapex_param_export.h>
#include <csapex/utility/any.h>


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
    ValueParameter(const std::string& name, const ParameterDescription& description, T def) : ValueParameter(name, description)
    {
        def_ = def;
    }

    virtual ~ValueParameter();

    const std::type_info& type() const override;
    std::string toStringImpl() const override;

    bool cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    template <typename T>
    T def() const
    {
        return read<T>(def_);
    }

protected:
    void get_unsafe(std::any& out) const override;
    bool set_unsafe(const std::any& v) override;

private:
    template <typename T>
    T read(const std::any& var) const
    {
        try {
            return std::any_cast<T>(var);

        } catch (const std::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof ValueParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::any value_;
    std::any def_;
};

template <>
inline std::string serializationName<ValueParameter>()
{
    return "value";
}

}  // namespace param
}  // namespace csapex

#endif  // VALUE_PARAMETER_H
