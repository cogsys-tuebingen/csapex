#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <mutex>

namespace csapex
{

class CSAPEX_EXPORT Parameterizable
{
public:
    typedef std::vector<std::pair<csapex::param::ParameterWeakPtr, std::function<void(csapex::param::Parameter *)> > > ChangedParameterList;

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
    void addConditionalParameter(const csapex::param::ParameterPtr& param, std::function<bool()> enable_condition, T& target)
    {
        addConditionalParameter(param, enable_condition, [&](csapex::param::Parameter* p) { target = p->as<T>(); });
    }

    void addConditionalParameter(const csapex::param::ParameterPtr& param, bool& condition_variable);
    void addConditionalParameter(const csapex::param::ParameterPtr& param, bool& condition_variable, std::function<void(csapex::param::Parameter *)> cb);

    template <typename T>
    void addConditionalParameter(const csapex::param::ParameterPtr& param, bool& condition_variable, T& target)
    {
        addConditionalParameter(param, condition_variable, [&](csapex::param::Parameter* p) { target = p->as<T>(); });
    }


    void addHiddenParameter(const csapex::param::ParameterPtr& param);
    void addHiddenParameter(const csapex::param::ParameterPtr& param, std::function<void(csapex::param::Parameter *)> cb);

    template <typename T>
    void addHiddenParameter(const csapex::param::ParameterPtr& param, T& target)
    {
        addHiddenParameter(param, [&](csapex::param::Parameter* p) { target = p->as<T>(); });
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
    template <typename T>
    void setParameterLater(const std::string& name, const T& value)
    {
        doSetParameterLater<T>(name, value);
    }

    void setSilent(bool silent);

    /***
     *  PARAMETER CONSTRAINTS
     */
    void addParameterCallback(csapex::param::ParameterPtr param, std::function<void(csapex::param::Parameter *)> cb);
    void addParameterCondition(csapex::param::ParameterPtr param, std::function<bool()> enable_condition);
    void addParameterCondition(csapex::param::ParameterPtr param, bool& enable_condition);

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

    csapex::param::ParameterPtr getMappedParameter(const std::string& name) const;

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
    virtual void setParameterState(MementoPtr memento);

private:
    template <typename T>
    T doReadParameter(const std::string& name) const;
    template <typename T>
    void doSetParameter(const std::string& name, const T& value);

    template <typename T>
    void doSetParameterLater(const std::string& name, const T& value)
    {
        {
            std::unique_lock<std::recursive_mutex> lock(changed_params_mutex_);
            param_updates_[name] = [this, name, value](){
                doSetParameter(name, value);
            };
        }
        parameters_changed();
    }

private:
    void parameterChanged(param::ParameterPtr param);
    void parameterChanged(param::ParameterPtr param, std::function<void(csapex::param::Parameter *)> cb);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    std::map<csapex::param::Parameter*, std::vector<csapex::slim_signal::Connection> > connections_;
    std::map<csapex::param::ParameterWeakPtr, std::function<bool()>, std::owner_less<csapex::param::ParameterWeakPtr>> conditions_;

    mutable std::recursive_mutex mutex_;
    mutable std::recursive_mutex changed_params_mutex_;
    std::map<std::string, std::function<void()>> param_updates_;
    ChangedParameterList changed_params_;

protected:
    GenericStatePtr parameter_state_;

private:
    bool silent_;
};

}

#endif // PARAMETERIZABLE_H
