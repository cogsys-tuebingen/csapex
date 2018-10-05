#ifndef PARAMETER_TEMPLATE_H
#define PARAMETER_TEMPLATE_H

/// PROJECT
#include <csapex/param/parameter_impl.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace param
{
template <typename Type, typename Instance>
class ParameterTemplate : public ParameterImplementation<Instance>
{
public:
    using Parent = ParameterImplementation<Instance>;
    using Self = ParameterTemplate<Type, Instance>;

    const Type& getValue() const
    {
        auto l = Parameter::lock();
        return value_;
    }

    const std::type_info& type() const override
    {
        auto l = Parameter::lock();
        return typeid(value_);
    }

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override
    {
        Parameter::serialize(data, version);
        data << value_;
    }

    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override
    {
        Parameter::deserialize(data, version);
        data >> value_;
    }

    void doSerialize(YAML::Node& n) const override
    {
        n["value"] = value_;
    }

    void doDeserialize(const YAML::Node& n) override
    {
        if (!n["value"].IsDefined()) {
            return;
        }

        value_ = n["value"].as<Type>();
    }

    bool hasData(const std::type_info& type) const override
    {
        return type == typeid(Type);
    }

    bool cloneDataFrom(const Clonable& other) override
    {
        if (const Instance* other_instance = dynamic_cast<const Instance*>(&other)) {
            if (value_ != other_instance->value_) {
                *this = *other_instance;
                Parameter::triggerChange();
            }
            return true;

        } else if (other.hasData(typeid(Type))) {
            value_ = *other.getDataPtr<Type>();
            return true;
        }

        return false;
    }

protected:
    ParameterTemplate() : Parent("noname", ParameterDescription())
    {
    }
    explicit ParameterTemplate(const std::string& name, const ParameterDescription& description, Type value) : Parent(name, description), value_(value)
    {
    }

    const void* getDataPtrUnsafe(const std::type_info& type) const override
    {
        if (type == typeid(Type)) {
            const auto* data_ptr = &value_;
            return static_cast<const void*>(data_ptr);
        }

        throw std::logic_error(std::string("invalid access to data of type ") + type2name(type));
    }

    void get_unsafe(boost::any& out) const
    {
        out = value_;
    }

    bool set_unsafe(const boost::any& v) override
    {
        Type a = boost::any_cast<Type>(v).getData();

        if (a != value_.getDataRef()) {
            value_ = a;
            return true;
        }

        return false;
    }

    std::string toStringImpl() const override
    {
        std::stringstream ss;
        ss << "[" << serializationName<Instance>() << ": " << value_ << "]";
        return ss.str();
    }

protected:
    Type value_;
};

}  // namespace param

}  // namespace csapex

#endif  // PARAMETER_TEMPLATE_H
