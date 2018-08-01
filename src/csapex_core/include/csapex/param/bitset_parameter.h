#ifndef BITSET_PARAMETER_H
#define BITSET_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_impl.hpp>
#include <csapex_param_export.h>

/// SYSTEM
#include <vector>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT BitSetParameter : public ParameterImplementation<BitSetParameter>
{
public:
    typedef std::shared_ptr<BitSetParameter> Ptr;

public:
    BitSetParameter();
    explicit BitSetParameter(const std::string& name, const ParameterDescription& description, int def = 0);
    virtual ~BitSetParameter();

    virtual const std::type_info& type() const override;
    virtual std::string toStringImpl() const override;

    void cloneDataFrom(const Clonable& other) override;

    void doSerialize(YAML::Node& e) const override;
    void doDeserialize(const YAML::Node& n) override;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    virtual bool accepts(const std::type_info& type) const override;

    int def() const
    {
        return def_;
    }

    void setBitSet(const std::map<std::string, int>& set);
    std::map<std::string, int> getBitSet() const;

    void clear();
    void setBits(const std::vector<std::string>& elements, bool silent = false);

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
    virtual void get_unsafe(boost::any& out) const override;
    virtual bool set_unsafe(const boost::any& v) override;

private:
    int value_;
    std::map<std::string, int> set_;
    int def_;
};


template <>
inline std::string serializationName<BitSetParameter>()
{
    return "bitset";
}

}  // namespace param
}  // namespace csapex

#endif  // BITSET_PARAMETER_H
