#ifndef PATH_PARAMETER_H
#define PATH_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/type_traits.hpp>

namespace param {


class PathParameter : public Parameter
{
    friend class boost::serialization::access;
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<PathParameter> Ptr;


public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const PathParameter& p) {
        p.doWrite(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const PathParameter::Ptr& p) {
        p->doWrite(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::PathParameter& value) {
        value.doRead(node);
    }

    friend void operator >> (const YAML::Node& node, param::PathParameter::Ptr& value) {
        if(!value) {
            value.reset(new PathParameter("loading", ParameterDescription(""), "", 0, false, false));
        }
        value->doRead(node);
    }

public:
    PathParameter();
    explicit PathParameter(const std::string& name, const ParameterDescription& description, const std::string& filter, bool is_file, bool input, bool output);
    virtual ~PathParameter();

    virtual int ID() const { return 0x004; }
    virtual std::string TYPE() const { return "Path"; }

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void doSetValueFrom(const Parameter& other);
    void doClone(const Parameter& other);

    void doWrite(YAML::Emitter& e) const;
    void doRead(const YAML::Node& n);

    template <typename T>
    T def() const { return read<T>(def_); }

    std::string filter() const;
    bool isFile() const;
    bool isInput() const;
    bool isOutput() const;

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

private:
    template <typename T>
    T read(const variant& var) const
    {
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof PathParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

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
