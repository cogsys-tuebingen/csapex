#ifndef COLOR_PARAMETER_H
#define COLOR_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT ColorParameter : public ParameterImplementation<ColorParameter, 0x002>
{
public:
    typedef std::shared_ptr<ColorParameter> Ptr;

public:
    ColorParameter();
    explicit ColorParameter(const std::string& name, const ParameterDescription& description, int r, int g, int b);

    virtual const std::type_info& type() const override;

    virtual std::string TYPE() const override
    {
        return "color";
    }

    virtual std::string toStringImpl() const override;

    void cloneDataFrom(const Clonable& other) override;
    void set(const std::vector<int>& v);

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::vector<int> def() const;
    std::vector<int> value() const;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    std::vector<int> colors_;
    std::vector<int> def_;
};

}  // namespace param
}  // namespace csapex

#endif  // COLOR_PARAMETER_H
