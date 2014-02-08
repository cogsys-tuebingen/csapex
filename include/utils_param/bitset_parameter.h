#ifndef BITSET_PARAMETER_H
#define BITSET_PARAMETER_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <vector>

namespace param {

class BitSetParameter : public Parameter
{
    friend class ParameterFactory;

public:
    typedef boost::shared_ptr<BitSetParameter> Ptr;

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const BitSetParameter& p) {
        p.write(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, const BitSetParameter::Ptr& p) {
        p->write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, param::BitSetParameter& value) {
        value.read(node);
    }    template<class Archive>

    friend void operator >> (const YAML::Node& node, param::BitSetParameter::Ptr& value) {
        if(!value) {
            value.reset(new BitSetParameter("loading"));
        }
        value->read(node);
    }

public:
    BitSetParameter();
    explicit BitSetParameter(const std::string& name);
    virtual ~BitSetParameter();

    virtual const std::type_info &type() const;
    virtual std::string toStringImpl() const;

    void setFrom(const Parameter& other);

    void write(YAML::Emitter& e) const;
    void read(const YAML::Node& n);

    int def() const { return def_; }

    void setBitSet(const std::map<std::string, int>& set);

    void clear();
    void setBits(const std::vector<std::string> &elements, bool silent = false);

    void setBitTo(const std::string& element, bool set, bool silent = false);
    void setBit(const std::string& element, bool silent = false);
    void clearBit(const std::string& element, bool silent = false);

    bool isSet(const std::string& element) const;

    void setByName(const std::string& name);

    std::string getName(int idx) const;
    std::string getName() const;

    int noParameters() const;

    bool isSelected(const std::string& name) const;
    bool setSelected(const std::string& name) const;

protected:
    virtual boost::any get_unsafe() const;
    virtual void set_unsafe(const boost::any& v);


private:
    int value_;
    std::map<std::string, int> set_;
    int def_;
};

}

#endif // BITSET_PARAMETER_H
