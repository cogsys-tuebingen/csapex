#ifndef PATH_PARAMETER_H
#define PATH_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex {
namespace param {

class CSAPEX_PARAM_EXPORT PathParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<PathParameter> Ptr;

public:
    PathParameter();
    explicit PathParameter(const std::string& name, const ParameterDescription& description, const std::string& filter, bool is_file, bool input, bool output);
    virtual ~PathParameter();

    virtual int ID() const override { return 0x004; }
    virtual std::string TYPE() const override { return "path"; }

    virtual const std::type_info &type() const override;
    virtual std::string toStringImpl() const override;

    void doSetValueFrom(const Parameter& other) override;
    void doClone(const Parameter& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

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

}
}

#endif // PATH_PARAMETER_H
