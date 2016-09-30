#ifndef OUTPUT_PROGRESS_PARAMETER_H
#define OUTPUT_PROGRESS_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT OutputProgressParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<OutputProgressParameter> Ptr;

public:
    OutputProgressParameter();
    explicit OutputProgressParameter(const std::string& name, const ParameterDescription& description);
    virtual ~OutputProgressParameter();

    virtual int ID() const override { return 0x009; }
    virtual std::string TYPE() const override { return "progress"; }

    void setProgress(int progress, int maximum);
    double getProgress() const;
    double getProgressMaximum() const;

protected:
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;
    virtual std::string toStringImpl() const override;

    virtual void doSerialize(YAML::Node& n) const override;
    virtual void doDeserialize(const YAML::Node& n) override;

    virtual void doSetValueFrom(const Parameter& other) override;
    virtual void doClone(const Parameter& other) override;

private:
    int value;
    int maximum;
};

}
}

#endif // OUTPUT_PROGRESS_PARAMETER_H

