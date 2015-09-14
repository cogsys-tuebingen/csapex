#ifndef PATH_PARAMETER_H
#define PATH_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter.h>

namespace param {


class PathParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef std::shared_ptr<PathParameter> Ptr;

public:
    PathParameter();
    explicit PathParameter(const std::string& name, const ParameterDescription& description, const std::string& filter, bool is_file, bool input, bool output);
    virtual ~PathParameter();

    virtual int ID() const { return 0x004; }
    virtual std::string TYPE() const { return "path"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doSerialize(YAML::Node& e) const;
    void doDeserialize(const YAML::Node& n);

    std::string def() const;

    std::string filter() const;
    bool isFile() const;
    bool isInput() const;
    bool isOutput() const;

protected:
    virtual boost::any get_unsafe() const override;
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

#endif // PATH_PARAMETER_H
