#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <utils_param/param_fwd.h>
#include <utils_param/parameter.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/contains.hpp>
#include <mutex>

namespace csapex
{

class Parameterizable
{
public:
    typedef std::vector<std::pair<param::Parameter*, std::function<void(param::Parameter *)> > > ChangedParameterList;

    typedef boost::mpl::vector<bool, int, double,
                                std::string,
                                std::pair<int,int>,
                                std::pair<double,double>,
                                std::vector<int>,
                                std::vector<double>
                              >
    SupportedTemplateParameters;

public:
    boost::signals2::signal<void()> parameters_changed;

public:
    Parameterizable();
    virtual ~Parameterizable();

    /***
     *  ADDING PARAMETERS
     */
    void addParameter(const param::ParameterPtr& param);
    void addParameter(const param::ParameterPtr& param, std::function<void(param::Parameter *)> cb);

    template <typename T>
    void addParameter(const param::ParameterPtr& param, T& target)
    {
        addParameter(param, [&](param::Parameter* p) { target = p->as<T>(); });
    }

    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition);
    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition, std::function<void(param::Parameter *)> cb);

    template <typename T>
    void addConditionalParameter(const param::ParameterPtr& param, std::function<bool()> enable_condition,T& target)
    {
        addConditionalParameter(param, enable_condition, [&](param::Parameter* p) { target = p->as<T>(); });
    }


    void addPersistentParameter(const param::ParameterPtr& param);

    void addTemporaryParameter(const param::ParameterPtr& param);
    void addTemporaryParameter(const param::ParameterPtr& param, std::function<void(param::Parameter *)> cb);
    void removeTemporaryParameter(const param::ParameterPtr& param);

    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param);
    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param, std::function<void(param::Parameter *)> cb);

    /***
     *  GETTING PARAMETERS
     */
    template <typename T>
    typename std::enable_if<boost::mpl::contains< SupportedTemplateParameters, T >::value, T>::type
    readParameter(const std::string& name) const
    {
        return doReadParameter<T>(name);
    }
    template <typename T>
    typename std::enable_if<!boost::mpl::contains< SupportedTemplateParameters, T >::value, T>::type
    readParameter(const std::string& name) const
    {
        throw std::runtime_error(std::string("cannot read parameter ") + name);
    }

    template <typename T>
    typename std::enable_if<boost::mpl::contains< SupportedTemplateParameters, T >::value, void>::type
    setParameter(const std::string& name, const T& value)
    {
        doSetParameter<T>(name, value);
    }
    template <typename T>
    typename std::enable_if<!boost::mpl::contains< SupportedTemplateParameters, T >::value, void>::type
    setParameter(const std::string& name, const T& /*value*/)
    {
        throw std::runtime_error(std::string("cannot set parameter ") + name);
    }

    /***
     *  PARAMETER CONSTRAINTS
     */
    void addParameterCallback(param::Parameter* param, std::function<void(param::Parameter *)> cb);
    void addParameterCondition(param::Parameter* param, std::function<bool()> enable_condition);

    void removeParameterCallbacks(param::Parameter* param);

    std::vector<param::ParameterPtr> getParameters() const;
    std::size_t getParameterCount() const;

    param::ParameterPtr getParameter(const std::string& name) const;
    template <typename T>
    typename T::Ptr getParameter(const std::string& name) const
    {
        return std::dynamic_pointer_cast<T> (getParameter(name));
    }

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


    template <typename T>
    T doReadParameter(const std::string& name) const;
    template <typename T>
    void doSetParameter(const std::string& name, const T& value);

private:
    void parameterChanged(param::Parameter* param);
    void parameterChanged(param::Parameter* param, std::function<void(param::Parameter *)> cb);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    std::map<param::Parameter*, std::vector<boost::signals2::connection> > connections_;
    std::map<param::Parameter*, std::function<bool()> > conditions_;

    mutable std::mutex changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, std::function<void(param::Parameter *)> > > changed_params_;

protected:
    GenericStatePtr parameter_state_;
};

}

#endif // PARAMETERIZABLE_H
