#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <utils_param/param_fwd.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <boost/signals2.hpp>

namespace csapex
{

class Parameterizable
{
public:
    typedef std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > ChangedParameterList;

public:
    Parameterizable();
    virtual ~Parameterizable();

    /***
     *  ADDING PARAMETERS
     */
    void addParameter(const param::ParameterPtr& param);
    void addParameter(const param::ParameterPtr& param, boost::function<void(param::Parameter *)> cb);

    void addConditionalParameter(const param::ParameterPtr& param, boost::function<bool()> enable_condition);
    void addConditionalParameter(const param::ParameterPtr& param, boost::function<bool()> enable_condition, boost::function<void(param::Parameter *)> cb);

    void addPersistentParameter(const param::ParameterPtr& param);

    void addTemporaryParameter(const param::ParameterPtr& param);
    void addTemporaryParameter(const param::ParameterPtr& param, boost::function<void(param::Parameter *)> cb);

    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param);
    void setTemporaryParameters(const std::vector<param::ParameterPtr>& param, boost::function<void(param::Parameter *)> cb);

    /***
     *  GETTING PARAMETERS
     */
    template <typename T>
    T readParameter(const std::string& name) const;

    template <typename T>
    void setParameter(const std::string& name, const T& value);

    /***
     *  PARAMETER CONSTRAINTS
     */
    void addParameterCallback(param::Parameter* param, boost::function<void(param::Parameter *)> cb);
    void addParameterCondition(param::Parameter* param, boost::function<bool()> enable_condition);

    void removeParameterCallbacks(param::Parameter* param);

    std::vector<param::ParameterPtr> getParameters() const;
    std::size_t getParameterCount() const;

    param::ParameterPtr getParameter(const std::string& name) const;
    template <typename T>
    typename T::Ptr getParameter(const std::string& name) const
    {
        return boost::dynamic_pointer_cast<T> (getParameter(name));
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


private:
    void parameterChanged(param::Parameter* param);
    void parameterChanged(param::Parameter* param, boost::function<void(param::Parameter *)> cb);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    std::map<param::Parameter*, std::vector<boost::signals2::connection> > connections_;
    std::map<param::Parameter*, boost::function<bool()> > conditions_;

    boost::mutex* changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params_;

protected:
    GenericStatePtr parameter_state_;
};

}

#endif // PARAMETERIZABLE_H
