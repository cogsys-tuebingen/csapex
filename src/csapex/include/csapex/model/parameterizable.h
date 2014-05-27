#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <utils_param/parameter.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <QMutex>

namespace csapex
{

class Parameterizable : public param::Parameter::access
{
public:
    typedef std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > ChangedParameterList;

public:
    Parameterizable();

    /***
     *  ADDING PARAMETERS
     */
    void addParameter(const param::Parameter::Ptr& param);
    void addParameter(const param::Parameter::Ptr& param, boost::function<void(param::Parameter *)> cb);

    void addConditionalParameter(const param::Parameter::Ptr& param, boost::function<bool()> enable_condition);
    void addConditionalParameter(const param::Parameter::Ptr& param, boost::function<bool()> enable_condition, boost::function<void(param::Parameter *)> cb);

    void addTemporaryParameter(const param::Parameter::Ptr& param);
    void addTemporaryParameter(const param::Parameter::Ptr& param, boost::function<void(param::Parameter *)> cb);

    void setTemporaryParameters(const std::vector<param::Parameter::Ptr>& param);
    void setTemporaryParameters(const std::vector<param::Parameter::Ptr>& param, boost::function<void(param::Parameter *)> cb);

    /***
     *  GETTING PARAMETERS
     */
    template <typename T>
    const T param(const std::string& name) const
    {
        return parameter_state_.param<T>(name);
    }

    std::vector<param::Parameter::Ptr> getParameters() const;

    param::Parameter::Ptr getParameter(const std::string& name) const;
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

    boost::shared_ptr<QMutexLocker> getParamLock();
    ChangedParameterList getChangedParameters();

protected:
    void addParameterCallback(param::Parameter *param, boost::function<void(param::Parameter *)> cb);
    void addParameterCondition(param::Parameter* param, boost::function<bool()> enable_condition);

private:
    void parameterChanged(param::Parameter* param);
    void parameterChanged(param::Parameter* param, boost::function<void(param::Parameter *)> cb);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    std::vector<boost::signals2::connection> connections_;
    std::map<param::Parameter*, boost::function<bool()> > conditions_;

    QMutex changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params_;

protected:
    GenericState parameter_state_;
};

}

#endif // PARAMETERIZABLE_H
