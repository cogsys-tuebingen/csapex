#ifndef OUTPUT_PROGRESS_PARAMETER_H
#define OUTPUT_PROGRESS_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

namespace param {


class OutputProgressParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<OutputProgressParameter> Ptr;

public:
    OutputProgressParameter();
    explicit OutputProgressParameter(const std::string& name, const ParameterDescription& description);
    virtual ~OutputProgressParameter();

    virtual int ID() const { return 0x009; }
    virtual std::string TYPE() const { return "progress"; }

    void setProgress(int progress, int maximum);
    double getProgress() const;
    double getProgressMaximum() const;

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);
    virtual std::string toStringImpl() const;

    virtual void doSerialize(YAML::Node& n) const;
    virtual void doDeserialize(const YAML::Node& n);

    virtual void doSetValueFrom(const Parameter& other);
    virtual void doClone(const Parameter& other);

private:
    int value;
    int maximum;
};

}
#endif // OUTPUT_PROGRESS_PARAMETER_H

