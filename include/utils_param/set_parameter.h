#ifndef SET_PARAMETER_H
#define SET_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <vector>

namespace param {

class SetParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef boost::any variant;

public:
    typedef boost::shared_ptr<SetParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const SetParameter& p) {
        p.write(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const SetParameter::Ptr& p) {
        p->write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::SetParameter& value) {
        value.read(node);
    }    template<class Archive>

    friend void operator >> (const YAML::Node& node, param::SetParameter::Ptr& value) {
        if(!value) {
            value.reset(new SetParameter("loading"));
        }
        value->read(node);
    }

public:
    SetParameter();
    explicit SetParameter(const std::string& name);
    virtual ~SetParameter();

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    template <typename T>
    T def() const { return read<T>(def_); }

    template <typename T>
    void setSet(const std::vector< std::pair<std::string, T> >& set) {
        set_.clear();
        for(typename std::vector< std::pair<std::string, T> >::const_iterator it = set.begin(); it != set.end(); ++it) {
            variant p = it->second;
            set_.push_back(std::make_pair(it->first, p));
        }
    }

    void setByName(const std::string& name);

    int noParameters() const;
    std::string getName(int idx) const;
    std::string getName() const;

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);

    std::string convertToString(const variant& v) const;

private:
    template <typename T>
    T read(const variant& var) const
    {
        try {
            return boost::any_cast<T> (var);

        } catch(const boost::bad_any_cast& e) {
            throw std::logic_error(std::string("typeof SetParameter is not ") + typeid(T).name() + ": " + e.what());
        }
    }

private:
    variant value_;
    std::vector< std::pair<std::string, variant> > set_;
    variant def_;
};

}

#endif // SET_PARAMETER_H
