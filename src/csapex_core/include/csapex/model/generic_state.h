#ifndef GENERIC_STATE_H
#define GENERIC_STATE_H

/// COMPONENT
#include <csapex/serialization/serializable.h>
#include <csapex/utility/uuid.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/model_fwd.h>

/// PROJECT
#include <csapex/param/param_fwd.h>
#include <csapex/utility/slim_signal.h>

/// SYSTEM
#include <set>
#include <yaml-cpp/yaml.h>

namespace csapex
{
class CSAPEX_CORE_EXPORT GenericState : public Serializable
{
public:
    typedef std::shared_ptr<GenericState> Ptr;

public:
    GenericState();
    GenericState(const GenericState& copy);
    GenericState(GenericState&& move);

    void operator=(const GenericState& copy);

    void setParentUUID(const UUID& parent_uuid);

    void addParameter(csapex::param::ParameterPtr param);
    void removeParameter(csapex::param::ParameterPtr param);

    void addPersistentParameter(const csapex::param::ParameterPtr& param);
    void removePersistentParameter(const csapex::param::ParameterPtr& param);
    void removePersistentParameters();

    void addTemporaryParameter(const csapex::param::ParameterPtr& param);
    void removeTemporaryParameter(const csapex::param::ParameterPtr& param);
    void removeTemporaryParameters();

    void setParameterSetSilence(bool silent);
    void triggerParameterSetChanged();

    csapex::param::Parameter& operator[](const std::string& name) const;
    csapex::param::ParameterPtr getParameter(const std::string& name) const;
    csapex::param::ParameterPtr getMappedParameter(const std::string& name) const;
    std::vector<csapex::param::ParameterPtr> getParameters() const;
    std::vector<csapex::param::ParameterPtr> getTemporaryParameters() const;
    std::vector<csapex::param::ParameterPtr> getPersistentParameters() const;

    std::size_t getParameterCount() const;
    bool hasParameter(const std::string& name) const;

    template <typename T>
    T readParameter(const std::string& name) const;

    ClonablePtr makeEmptyInstance() const override;
    void cloneDataFrom(const Clonable& other) override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    void setFrom(const GenericState& rhs);

    void writeYaml(YAML::Node& out) const;
    void readYaml(const YAML::Node& node);

    void initializePersistentParameters();

private:
    void registerParameter(const csapex::param::ParameterPtr& param);
    void unregisterParameter(const csapex::param::ParameterPtr& param);

public:
    UUID parent_uuid_;

    std::map<std::string, csapex::param::ParameterPtr> params;
    std::map<std::string, std::string> param_valid_name_cache;
    std::map<std::string, bool> temporary;
    std::map<std::string, bool> cached_parameter;
    std::set<std::string> persistent;
    std::set<std::string> legacy;
    std::vector<std::string> order;

    bool silent_;

    slim_signal::Signal<void()> parameter_set_changed;
    slim_signal::Signal<void(csapex::param::ParameterPtr)> legacy_parameter_added;
    slim_signal::Signal<void(param::ParameterPtr)> parameter_added;
    slim_signal::Signal<void(param::Parameter*)> parameter_changed;
    slim_signal::Signal<void(param::ParameterPtr)> parameter_removed;
};

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_CORE_EXPORT convert<csapex::GenericState>
{
    static Node encode(const csapex::GenericState& rhs);
    static bool decode(const Node& node, csapex::GenericState& rhs);
};
}  // namespace YAML

#endif  // GENERIC_STATE_H
