#ifndef PATH_PARAMETER_H
#define PATH_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT PathParameter : public ParameterImplementation<PathParameter>
{
public:
    typedef std::shared_ptr<PathParameter> Ptr;

public:
    PathParameter();
    explicit PathParameter(const std::string& name, const ParameterDescription& description, const std::string& filter, bool is_file, bool input, bool output);
    virtual ~PathParameter();

    virtual const std::type_info& type() const override;
    virtual std::string toStringImpl() const override;

    bool cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::string def() const;

    std::string filter() const;
    bool isFile() const;
    bool isInput() const;
    bool isOutput() const;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    std::string value_;
    std::string def_;

    std::string filter_;
    bool is_file_;
    bool input_;
    bool output_;
};

template <>
inline std::string serializationName<PathParameter>()
{
    return "path";
}

}  // namespace param
}  // namespace csapex

#endif  // PATH_PARAMETER_H
