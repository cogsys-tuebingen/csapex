#ifndef OUTPUT_PROGRESS_PARAMETER_H
#define OUTPUT_PROGRESS_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_core/csapex_param_export.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT OutputProgressParameter : public ParameterImplementation<OutputProgressParameter>
{
public:
    typedef std::shared_ptr<OutputProgressParameter> Ptr;

public:
    OutputProgressParameter();
    explicit OutputProgressParameter(const std::string& name, const ParameterDescription& description);
    virtual ~OutputProgressParameter();

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    void advanceProgress(int step = 1);
    void setProgress(int progress, int maximum);
    double getProgress() const;
    double getProgressMaximum() const;

protected:
    void get_unsafe(boost::any& out) const override;
    bool set_unsafe(const boost::any& v) override;
    std::string toStringImpl() const override;

    void doSerialize(YAML::Node& n) const override;
    void doDeserialize(const YAML::Node& n) override;

    bool cloneDataFrom(const Clonable& other) override;

private:
    int value;
    int maximum;
};

template <>
inline std::string serializationName<OutputProgressParameter>()
{
    return "progress";
}

}  // namespace param
}  // namespace csapex

#endif  // OUTPUT_PROGRESS_PARAMETER_H
