#ifndef STRING_LIST_PARAMETER_H
#define STRING_LIST_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_core/csapex_param_export.h>

/// SYSTEM
#include <boost/any.hpp>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT StringListParameter : public ParameterImplementation<StringListParameter>
{
public:
    typedef std::shared_ptr<StringListParameter> Ptr;

public:
    StringListParameter();
    explicit StringListParameter(const std::string& name, const ParameterDescription& description);
    virtual ~StringListParameter();

    void add(const std::string& value);
    void setAt(std::size_t i, const std::string& value);
    void remove(std::size_t i);
    void removeAll(const std::string& value);

    std::size_t count() const;
    std::vector<std::string> getValues() const;

    virtual const std::type_info& type() const override;
    virtual std::string toStringImpl() const override;

    bool cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

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
            throw std::logic_error(std::string("typeof StringListParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    std::vector<std::string> list_;
};

template <>
inline std::string serializationName<StringListParameter>()
{
    return "string_list";
}

}  // namespace param
}  // namespace csapex

#endif  // STRING_LIST_PARAMETER_H
