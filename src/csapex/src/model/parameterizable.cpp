/// HEADER
#include <csapex/model/parameterizable.h>


using namespace csapex;


Parameterizable::Parameterizable()
{

}

void Parameterizable::addParameterCallback(param::Parameter* param, boost::function<void(param::Parameter *)> cb)
{
    connections_.push_back(parameter_changed(*param).connect(boost::bind(&Parameterizable::parameterChanged, this, _1, cb)));
    if(param->hasState()) {
        parameterChanged(param, cb);
    }
}

void Parameterizable::addParameterCondition(param::Parameter* param, boost::function<bool ()> enable_condition)
{
    conditions_[param] = enable_condition;
}

void Parameterizable::parameterChanged(param::Parameter *)
{
    if(!conditions_.empty()) {
        checkConditions(false);
    }
}

void Parameterizable::parameterChanged(param::Parameter *param, boost::function<void(param::Parameter *)> cb)
{
    QMutexLocker lock(&changed_params_mutex_);
    changed_params_.push_back(std::make_pair(param, cb));
}

void Parameterizable::parameterEnabled(param::Parameter */*param*/, bool /*enabled*/)
{
    triggerParameterSetChanged();
}



void Parameterizable::checkConditions(bool silent)
{
    bool change = false;
    setParameterSetSilence(true);
    for(std::map<param::Parameter*, boost::function<bool()> >::iterator it = conditions_.begin(); it != conditions_.end(); ++it) {
        param::Parameter* p = it->first;
        bool should_be_enabled = it->second();
        if(should_be_enabled != p->isEnabled()) {
            it->first->setEnabled(should_be_enabled);
            change = true;
        }
    }
    setParameterSetSilence(false);

    if(change && !silent) {
        triggerParameterSetChanged();
    }
}

void Parameterizable::addTemporaryParameter(const param::Parameter::Ptr &param)
{
    parameter_state_.addTemporaryParameter(param);
}

void Parameterizable::addTemporaryParameter(const param::Parameter::Ptr &param, boost::function<void (param::Parameter *)> cb)
{
    parameter_state_.addTemporaryParameter(param);
    addParameterCallback(param.get(), cb);
}

void Parameterizable::addParameter(const param::Parameter::Ptr &param)
{
    parameter_state_.addParameter(param);

    connections_.push_back(parameter_changed(*param).connect(boost::bind(&Parameterizable::parameterChanged, this, _1)));
    connections_.push_back(parameter_enabled(*param).connect(boost::bind(&Parameterizable::parameterEnabled, this, _1, _2)));
}

void Parameterizable::addParameter(const param::Parameter::Ptr &param, boost::function<void (param::Parameter *)> cb)
{
    addParameter(param);
    addParameterCallback(param.get(), cb);
}


void Parameterizable::addConditionalParameter(const param::Parameter::Ptr &param, boost::function<bool()> enable_condition)
{
    addParameter(param);
    addParameterCondition(param.get(), enable_condition);
}


void Parameterizable::addConditionalParameter(const param::Parameter::Ptr &param, boost::function<bool()> enable_condition, boost::function<void (param::Parameter *)> cb)
{
    addParameter(param);
    addParameterCallback(param.get(), cb);
    addParameterCondition(param.get(), enable_condition);
}


std::vector<param::Parameter::Ptr> Parameterizable::getParameters() const
{
    return parameter_state_.getParameters();
}

param::Parameter::Ptr Parameterizable::getParameter(const std::string &name) const
{
    return parameter_state_.getParameter(name);
}

bool Parameterizable::isParameterEnabled(const std::string &name) const
{
    return getParameter(name)->isEnabled();
}

void Parameterizable::setParameterEnabled(const std::string &name, bool enabled)
{
    getParameter(name)->setEnabled(enabled);
}



void Parameterizable::setParameterSetSilence(bool silent)
{
    parameter_state_.setParameterSetSilence(silent);
}

void Parameterizable::removeTemporaryParameters()
{
    // TODO: handle callbacks!
    parameter_state_.removeTemporaryParameters();
}

void Parameterizable::triggerParameterSetChanged()
{
    parameter_state_.triggerParameterSetChanged();
}

boost::shared_ptr<QMutexLocker> Parameterizable::getParamLock()
{
    return boost::shared_ptr<QMutexLocker>(new QMutexLocker(&changed_params_mutex_));
}

Parameterizable::ChangedParameterList Parameterizable::getChangedParameters()
{
    ChangedParameterList changed_params;

    QMutexLocker lock(&changed_params_mutex_);
    changed_params = changed_params_;
    changed_params_.clear();

    return changed_params;
}
