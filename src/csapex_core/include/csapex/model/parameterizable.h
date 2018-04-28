#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <mutex>

namespace csapex
{

/**
 * @brief The Parameterizable class represents an object that has parameters.
 */
class CSAPEX_CORE_EXPORT Parameterizable
{
public:
    /**
     * @brief ChangedParameterList A list of parameters and their callback functions
     */
    typedef std::vector<std::pair<param::ParameterWeakPtr, std::vector<std::function<void(param::Parameter *)>>>> ChangedParameterList;

public:
    /**
     * @brief parameters_changed is triggered, when one of the object's parameters has a new value.
     */
    slim_signal::Signal<void()> parameters_changed;

public:
    /**
     * @brief Parameterizable
     */
    Parameterizable();

    /**
     * @brief ~Parameterizable
     */
    virtual ~Parameterizable();


    /***
     *  ADDING PARAMETERS
     */
    /**
     * @brief addParameter adds a new parameter to the object that are kept until the object is destroyed.
     * @param[in] param The new parameter
     * @warning The name of the parameter has to be unique for the node.
     * @see addPersistentParameter to add parameters that are serialized with the object.
     * @see addTemporaryParameter to add parameters that can be removed during the lifetime of the object.
     */
    void addParameter(const param::ParameterPtr& param);
    /**
     * @brief addParameter adds a new parameter to the object and registers a callback function.
     * @param[in] param The new parameter
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCallback
     */
    void addParameter(const param::ParameterPtr& param, std::function<void(param::Parameter *)> cb);

    /**
     * @brief addParameter adds a new parameter to the object and synchronizes its value to a memory location.
     * @param[in] param The new parameter
     * @param[in] target A reference to where the value of the parameter should be written to,
     *            when it changes.
     * @warning The name of the parameter has to be unique for the node.
     */
    template <typename T, typename R = T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
    void addParameter(const param::ParameterPtr& param, T& target)
    {
        addParameter(param, [&](param::Parameter* p) { target = p->as<R>(); });
    }
    template <typename T, typename R = T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
    void addParameter(const param::ParameterPtr& param, T& target)
    {
        addParameter(param, [&](param::Parameter* p) { target = static_cast<R>(p->as<int>()); });
    }

    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied.
     * @param[in] param The new parameter
     * @param[in] enable_condition A function that determines visibility.
     *            The parameter is shown only if the function returns <b>true</b>.
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     */
    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition);

    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied, and registers a callback function.
     * @param[in] param The new parameter
     * @param[in] enable_condition A function that determines visibility.
     *            The parameter is shown only if the function returns <b>true</b>.
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     * @see addParameterCallback
     */
    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition, std::function<void(param::Parameter *)> cb);

    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied, and synchronizes its value to a memory location.
     * @param[in] param The new parameter
     * @param[in] enable_condition A function that determines visibility.
     *            The parameter is shown only if the function returns <b>true</b>.
     * @param[in] target A reference to where the value of the parameter should be written to,
     *            when it changes.
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     */
    template <typename T, typename R = T>
    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition, T& target)
    {
        addConditionalParameter(param, enable_condition, [&](param::Parameter* p) { target = p->as<R>(); });
    }

    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied.
     * @param[in] param The new parameter
     * @param[in] condition_variable A variable that determines visibility.
     *            The parameter is shown only if the variable is <b>true</b>.
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     */
    void addConditionalParameter(const param::ParameterPtr& param, bool& condition_variable);
    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied, and registers a callback function.
     * @param[in] param The new parameter
     * @param[in] condition_variable A variable that determines visibility.
     *            The parameter is shown only if the variable is <b>true</b>.
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     * @see addParameterCallback
     */
    void addConditionalParameter(const param::ParameterPtr& param, bool& condition_variable, std::function<void(param::Parameter *)> cb);

    /**
     * @brief addConditionalParameter adds a new parameter to the object, that is only
     *        shown to users when a condition is satisfied, and synchronizes its value to a memory location.
     * @param[in] param The new parameter
     * @param[in] condition_variable A variable that determines visibility.
     *            The parameter is shown only if the variable is <b>true</b>.
     * @param[in] target A reference to where the value of the parameter should be written to,
     *            when it changes.
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCondition
     */
    template <typename T, typename R = T>
    void addConditionalParameter(const param::ParameterPtr& param, bool& condition_variable, T& target)
    {
        addConditionalParameter(param, condition_variable, [&](param::Parameter* p) { target = p->as<R>(); });
    }


    /**
     * @brief addHiddenParameter adds a new parameter to the object that is not shown to users.
     * @param[in] param The new hidden parameter
     * @warning The name of the parameter has to be unique for the node.
     */
    void addHiddenParameter(const param::ParameterPtr& param);

    /**
     * @brief addHiddenParameter adds a new parameter to the object that is not shown to users.
     * @param[in] param The new hidden parameter
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameterCallback
     */
    void addHiddenParameter(const param::ParameterPtr& param, std::function<void(param::Parameter *)> cb);


    /**
     * @brief addHiddenParameter adds a new parameter to the object that is not shown to users.
     * @param[in] param The new hidden parameter
     * @param[out] target A reference to where the value of the parameter should be written to,
     *             when it changes.
     * @warning The name of the parameter has to be unique for the node.
     */
    template <typename T, typename R = T>
    void addHiddenParameter(const param::ParameterPtr& param, T& target)
    {
        addHiddenParameter(param, [&](param::Parameter* p) { target = p->as<R>(); });
    }

    /**
     * @brief addPersistentParameter adds a new parameter to the object that is serialized with the object.
     * @param[in] param The new persistent parameter
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameter to add parameters for the lifetime of the object.
     * @see addTemporaryParameter to add parameters that can be removed during the lifetime of the object.
     * @see removePersistentParameter, removePersistentParameters
     */
    void addPersistentParameter(const param::ParameterPtr& param);

    /**
     * @brief removePersistentParameter removes a persistent parameter
     * @param param copy of the parameter to be removed.
     * @see addPersistentParameter, removePersistentParameters
     */
    void removePersistentParameter(const param::ParameterPtr& param);

    /**
     * @brief removePersistentParameters removes all persistent paramters.
     * @see addPersistentParameter
     */
    void removePersistentParameters();

    /**
     * @brief addTemporaryParameter adds a new parameter to the object that can be
     *        removed during the lifetime with the object.
     * @param[in] param The new persistent parameter
     * @warning The name of the parameter has to be unique for the node.
     * @see addParameter to add parameters for the lifetime of the object.
     * @see addPersistentParameter to add parameters that are serialized with the object.
     * @see removeTemporaryParameter, removeTemporaryParameters, setTemporaryParameters
     */
    void addTemporaryParameter(const param::ParameterPtr& param);

    /**
     * @brief addTemporaryParameter adds a new parameter to the object that can be
     *        removed during the lifetime with the object. Also registers a callback function.
     * @param[in] param The new persistent parameter
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @warning The name of the parameter has to be unique for the node.
     * @see removeTemporaryParameter, removeTemporaryParameters, setTemporaryParameters
     * @see addParameterCallback
     */
    void addTemporaryParameter(const param::ParameterPtr& param, std::function<void(param::Parameter *)> cb);

    /**
     * @brief addTemporaryParameter adds a new parameter to the object that can be
     *        removed during the lifetime with the object.
     * @param[in] param The new hidden parameter
     * @param[out] target A reference to where the value of the parameter should be written to,
     *             when it changes.
     * @warning The name of the parameter has to be unique for the node.
     */
    template <typename T, typename R = T>
    void addTemporaryParameter(const param::ParameterPtr& param, T& target)
    {
        addTemporaryParameter(param, [&](param::Parameter* p) { target = p->as<R>(); });
    }


    /**
     * @brief removeTemporaryParameter removes a temporary parameter
     * @param param copy of the parameter to be removed.
     * @see addTemporaryParameter, removeTemporaryParameters, setTemporaryParameters
     */
    void removeTemporaryParameter(const param::ParameterPtr& param);

    /**
     * @brief removeTemporaryParameters removes all temporary parameters
     * @see addTemporaryParameter, removeTemporaryParameter
     */
    void removeTemporaryParameters();

    /**
     * @brief setTemporaryParameters replace all temporary parameters with new ones.
     * @param param[in] A vector of parameters to use as the new temporary parameters
     * @see addTemporaryParameter, removeTemporaryParameter, removeTemporaryParameters
     */
    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param);

    /**
     * @brief setTemporaryParameters replace all temporary parameters with new ones and set
     *        a common callback function for all of them
     * @param param[in] A vector of parameters to use as the new temporary parameters
     * @param[in] cb A callback function that is called whenever the parameter's value changes
     * @see addTemporaryParameter, removeTemporaryParameter, removeTemporaryParameters
     * @see addParameterCallback
     */
    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param, std::function<void(param::Parameter *)> cb);

    /***
     *  GETTING PARAMETERS
     */
    /**
     * @brief readParameter returns the values of a parameter
     * @param name unique name of the parameter for which to get the value
     * @return The value casted to <b>T</b>.
     */
    template <typename T>
    T readParameter(const std::string& name) const
    {
        return doReadParameter<T>(name);
    }

    /**
     * @brief setParameter directly updates the value of a parameter
     * @param name unique name of the parameter for which to get the value
     * @param value the value to use
     */
    template <typename T>
    void setParameter(const std::string& name, const T& value)
    {
        doSetParameter<T>(name, value);
    }
    /**
     * @brief setParameter updates the value of a parameter the next time processing
     *        power is available
     * @param name unique name of the parameter for which to get the value
     * @param value the value to use
     */
    template <typename T>
    void setParameterLater(const std::string& name, const T& value)
    {
        doSetParameterLater<T>(name, value);
    }

    /**
     * @brief setSilent dis-/enables signal emission.
     * @param silent while the node is silent, no functions emit signals.
     */
    void setSilent(bool silent);

    /***
     *  PARAMETER CONSTRAINTS
     */
    /**
     * @brief addParameterCallback adds a callback function this is called once the parameter changes.
     * @param [in] param the parameter to observe
     * @param [in] cb the callback to call
     */
    void addParameterCallback(param::ParameterPtr param, std::function<void(param::Parameter *)> cb);

    /**
     * @brief addParameterCondition adds a condition which has to be satisfied for the parameter
     *        to be visible for users.
     * @param [in] param the parameter
     * @param [in] enable_condition The parameter is shown only if the function returns <b>true</b>.
     */
    void addParameterCondition(param::ParameterPtr param, std::function<bool()> enable_condition);
    /**
     * @brief addParameterCondition adds a condition which has to be satisfied for the parameter
     *        to be visible for users.
     * @param [in] param the parameter
     * @param [in] condition_variable A variable that determines visibility.
     *        The parameter is shown only if the variable is <b>true</b>.
     */
    void addParameterCondition(param::ParameterPtr param, bool& enable_condition);

    /**
     * @brief removeParameterCallbacks removes all callbacks registered for a given parameter
     * @param param raw pointer to the parameter for which all callbacks should be removed
     */
    void removeParameterCallbacks(param::Parameter* param);

    /**
     * @brief getParameters
     * @return a vector of all parameters, including persistent and temporary parameters
     */
    std::vector<param::ParameterPtr> getParameters() const;

    /**
     * @brief getTemporaryParameters
     * @return a vector of all temporary parameters
     */
    std::vector<param::ParameterPtr> getTemporaryParameters() const;

    /**
     * @brief getPersistentParameters
     * @return a vector of all persistent parameters
     */
    std::vector<param::ParameterPtr> getPersistentParameters() const;

    /**
     * @brief getParameter access to a specific parameter
     * @param name the unique name of the parameter
     * @return a pointer to the requested parameter.
     * @throws if the parameter doesn't exist
     */
    param::ParameterPtr getParameter(const std::string& name) const;

    /**
     * @brief getParameter access to a specific parameter and casts it to the given type.
     * @param name the unique name of the parameter
     * @return a pointer to the requested parameter casted to an implementation class.
     * @throws if the parameter doesn't exist
     */
    template <typename T>
    typename T::Ptr getParameter(const std::string& name) const
    {
        return std::dynamic_pointer_cast<T> (getParameter(name));
    }

    /**
     * @brief getMappedParameter access to a specific parameter and casts it to the given type.
     *        Allows the name to be mapped to a valid name in the ROS conventions.
     *
     * @param name valid ROS name conversion of the parameters name
     * @return the parameter that corresponds the to valid ROS name
     *
     * @see getParameter
     */
    param::ParameterPtr getMappedParameter(const std::string& name) const;

    /**
     * @brief getParameterCount counts the number of existing parameters
     * @return the number of existing parameters
     */
    std::size_t getParameterCount() const;

    /**
     * @brief hasParameter checks for the existence of a parameter
     * @param name name of the parameter
     * @return <b>true</b>, iff a parameter with the name exists.
     */
    bool hasParameter(const std::string& name) const;

    /***
     *  MISC
     */
    /**
     * @brief checkConditions checks all parameter conditions and updates the parameters.
     * @param silent if <b>true</b>, no signals will be emitted
     */
    void checkConditions(bool silent);

    /**
     * @brief isParameterEnabled checks if a parameter is enabled
     *
     * Disabled parameters are not shown to the user.
     * Conditional parameters are disabled, when their condition is not met.
     *
     * @param name name of the parameter
     * @return <b>true</b>, iff the parameter is disabled
     */
    bool isParameterEnabled(const std::string& name) const;

    /**
     * @brief setParameterEnabled sets the enabled flag of a parameter.
     *
     * Disabled parameters are not shown to the user.
     * Conditional parameters are disabled, when their condition is not met.
     *
     * @param name name of the parameter
     * @param enabled specifies whether the parameter is enabled
     */
    void setParameterEnabled(const std::string& name, bool enabled);

    /**
     * @brief setParameterSetSilence suppresses all signals from being emitted
     * @param silent specifies, wheter signals should be suppressed.
     */
    void setParameterSetSilence(bool silent);

    /**
     * @brief triggerParameterSetChanged emits signals that were suppressed before.
     */
    void triggerParameterSetChanged();

    /**
     * @brief hasChangedParameters checks if there are parameters that have changed in value.
     * @return true, iff there are parameters with changed values
     */
    bool hasChangedParameters() const;

    /**
     * @brief getChangedParameters returns a list of all parameters that have changed
     *        since this function was called the last time.
     * @return list of changed parameters
     */
    ChangedParameterList getChangedParameters();

    /**
     * @brief getParameterState returns the underlying Memento
     * @return Pointer to the underlying state
     */
    virtual GenericStatePtr getParameterState();

    /**
     * @brief getParameterState returns a copy of the underlying Memento
     * @return A copy of the underlying state
     */
    virtual GenericStatePtr getParameterStateClone() const;

    /**
     * @brief setParameterState replace the underlying Memento
     * @param memento the state to use
     */
    virtual void setParameterState(MementoPtr memento);

private:
    template <typename T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
    T doReadParameter(const std::string& name) const;
    template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
    T doReadParameter(const std::string& name) const
    {
        return static_cast<T>(doReadParameter<int>(name));
    }

    template <typename T, typename std::enable_if<!std::is_enum<T>::value, int>::type = 0>
    void doSetParameter(const std::string& name, const T& value);
    template <typename T, typename std::enable_if<std::is_enum<T>::value, int>::type = 0>
    void doSetParameter(const std::string& name, const T& value)
    {
        doSetParameter(name, static_cast<int>(value));
    }

    template <typename T>
    void doSetParameterLater(const std::string& name, const T& value)
    {
        {
            std::unique_lock<std::recursive_mutex> lock(changed_params_mutex_);
            bool change = getParameter(name)->setSilent(value);
            if(change) {
                param_updates_[name] = [this, name, value](){
                    getParameter(name)->triggerChange();
                };
            }
        }
        parameters_changed();
    }

private:
    void parameterChanged(param::ParameterPtr param);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    std::map<param::Parameter*, std::vector<slim_signal::Connection> > parameter_connections_;
    std::map<param::ParameterWeakPtr, std::function<bool()>, std::owner_less<param::ParameterWeakPtr>> conditions_;

    mutable std::recursive_mutex mutex_;
    mutable std::recursive_mutex changed_params_mutex_;
    std::map<std::string, std::function<void()>> param_updates_;
    ChangedParameterList changed_params_;

    std::map<param::Parameter*, std::vector<std::function<void(param::Parameter*)>>> param_callbacks_;

protected:
    GenericStatePtr parameter_state_; ///< the underlying memento

private:
    bool silent_;
};

}

#endif // PARAMETERIZABLE_H
