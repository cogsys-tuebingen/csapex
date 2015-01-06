#ifndef GENERIC_STATE_H
#define GENERIC_STATE_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/utility/uuid.h>

/// PROJECT
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <boost/signals2.hpp>

namespace csapex
{

class GenericState : public Memento
{
public:
    typedef boost::shared_ptr<GenericState> Ptr;

public:
    GenericState();
    GenericState::Ptr clone() const;

    void setParentUUID(const UUID& parent_uuid);

    void addParameter(param::ParameterPtr param);
    void removeParameter(param::ParameterPtr param);

    void addPersistentParameter(const param::ParameterPtr& param);

    void addTemporaryParameter(const param::ParameterPtr& param);
    void removeTemporaryParameter(const param::ParameterPtr& param);
    void removeTemporaryParameters();

    void setParameterSetSilence(bool silent);
    void triggerParameterSetChanged();

    param::Parameter& operator [] (const std::string& name) const;
    param::ParameterPtr getParameter(const std::string& name) const;
    std::vector<param::ParameterPtr> getParameters() const;
    std::vector<param::ParameterPtr> getTemporaryParameters() const;

    std::size_t getParameterCount() const;

    template <typename T>
    T readParameter (const std::string& name) const;

    void setFrom(const GenericState& rhs);

    virtual void writeYaml(YAML::Node& out) const;
    virtual void readYaml(const YAML::Node& node);

    void initializePersistentParameters();

private:
    void registerParameter(const param::ParameterPtr &param);
    void unregisterParameter(const param::ParameterPtr &param);

public:
    UUID parent_uuid_;

    std::map<std::string, param::ParameterPtr> params;
    std::map<std::string, bool> temporary;
    std::set<std::string> persistent;
    std::vector<std::string> order;

    bool silent_;

    boost::shared_ptr<boost::signals2::signal<void()> > parameter_set_changed;
    boost::shared_ptr<boost::signals2::signal<void(param::Parameter*)> > parameter_added;
    boost::shared_ptr<boost::signals2::signal<void(param::ParameterPtr)> > parameter_removed;
};

}

#endif // GENERIC_STATE_H
