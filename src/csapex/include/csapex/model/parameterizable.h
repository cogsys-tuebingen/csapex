#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <mutex>

namespace csapex
{

class Parameterizable
{
public:
    typedef std::vector<std::pair<csapex::param::Parameter*, std::function<void(csapex::param::Parameter *)> > > ChangedParameterList;

public:
    csapex::slim_signal::Signal<void()> parameters_changed;

public:
    Parameterizable();
    virtual ~Parameterizable();

    /***
     *  ADDING PARAMETERS
     */
    void addParameter(const csapex::param::ParameterPtr& param);
    void addParameter(const csapex::param::ParameterPtr& param, std::function<void(csapex::param::Parameter *)> cb);

    template <typename T>
    void addParameter(const csapex::param::ParameterPtr& param, T& target)
    {
        addParameter(param, [&](csapex::param::Parameter* p) { target = p->as<T>(); });
    }

    void addConditionalParameter(const csapex::param::ParameterPtr& param, std::function<bool()> enable_condition);
    void addConditionalParameter(const csapex::param::ParameterPtr& param, std::function<bool()> enable_condition, std::function<void(csapex::param::Parameter *)> cb);

    template <typename T>
    void addConditionalParameter(const csapex::param::ParameterPtr& param, std::function<bool()> enable_condition,T& target)
    {
        addConditionalParameter(param, enable_condition, [&](csapex::param::Parameter* p) { target = p->as<T>(); });
    }


    void addPersistentParameter(const csapex::param::ParameterPtr& param);

    void addTemporaryParameter(const csapex::param::ParameterPtr& param);
    void addTemporaryParameter(const csapex::param::ParameterPtr& param, std::function<void(csapex::param::Parameter *)> cb);
    void removeTemporaryParameter(const csapex::param::ParameterPtr& param);

    void setTemporaryParameters(const std::vector<csapex::param::ParameterPtr>& param);
    void setTemporaryParameters(const std::vector<csapex::param::ParameterPtr>& param, std::function<void(csapex::param::Parameter *)> cb);

    /***
     *  GETTING PARAMETERS
     */
    template <typename T>
    T readParameter(const std::string& name) const
    {
        return doReadParameter<T>(name);
    }

    template <typename T>
    void setParameter(const std::string& name, const T& value)
    {
        doSetParameter<T>(name, value);
    }

    /***
     *  PARAMETER CONSTRAINTS
     */
    void addParameterCallback(csapex::param::Parameter* param, std::function<void(csapex::param::Parameter *)> cb);
    void addParameterCondition(csapex::param::Parameter* param, std::function<bool()> enable_condition);

    void removeParameterCallbacks(csapex::param::Parameter* param);

    std::vector<csapex::param::ParameterPtr> getParameters() const;
    std::vector<csapex::param::ParameterPtr> getTemporaryParameters() const;
    std::vector<csapex::param::ParameterPtr> getPersistentParameters() const;

    csapex::param::ParameterPtr getParameter(const std::string& name) const;
    template <typename T>
    typename T::Ptr getParameter(const std::string& name) const
    {
        return std::dynamic_pointer_cast<T> (getParameter(name));
    }

    std::size_t getParameterCount() const;
    bool hasParameter(const std::string& name) const;

    /***
     *  MISC
     */
    void checkConditions(bool silent);

    bool isParameterEnabled(const std::string& name) const;
    void setParameterEnabled(const std::string& name, bool enabled);

    void setParameterSetSilence(bool silent);
    void removeTemporaryParameters();
    void triggerParameterSetChanged();

    ChangedParameterList getChangedParameters();

    virtual GenericStatePtr getParameterState();
    virtual GenericStatePtr getParameterStateClone() const;
    virtual void setParameterState(Memento::Ptr memento);

private:
    template <typename T>
    T doReadParameter(const std::string& name) const;
    template <typename T>
    void doSetParameter(const std::string& name, const T& value);

private:
    void parameterChanged(csapex::param::Parameter* param);
    void parameterChanged(csapex::param::Parameter* param, std::function<void(csapex::param::Parameter *)> cb);
    void parameterEnabled(csapex::param::Parameter* param, bool enabled);

private:
    std::map<csapex::param::Parameter*, std::vector<csapex::slim_signal::Connection> > connections_;
    std::map<csapex::param::Parameter*, std::function<bool()> > conditions_;

    mutable std::mutex changed_params_mutex_;
    std::vector<std::pair<csapex::param::Parameter*, std::function<void(csapex::param::Parameter *)> > > changed_params_;

protected:
    GenericStatePtr parameter_state_;
};

}

#endif // PARAMETERIZABLE_H
