/// HEADER
#include <csapex/model/parameterizable.h>

/// PROJECT
#include <csapex/model/generic_state.h>
#include <csapex/utility/assert.h>
#include <csapex/param/parameter.h>

/// SYSTEM
#include <mutex>
#include <iostream>

using namespace csapex;


Parameterizable::Parameterizable()
    : parameter_state_(new GenericState), silent_(false)
{
}

Parameterizable::~Parameterizable()
{
}

void Parameterizable::setSilent(bool silent)
{
    silent_ = silent;
}

template <typename T>
T Parameterizable::doReadParameter(const std::string& name) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getParameter(name)->as<T>();
}

template <typename T>
void Parameterizable::doSetParameter(const std::string& name, const T& value)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->getParameter(name)->set<T>(value);
}


template CSAPEX_EXPORT bool Parameterizable::doReadParameter<bool>(const std::string& name) const;
template CSAPEX_EXPORT double Parameterizable::doReadParameter<double>(const std::string& name) const;
template CSAPEX_EXPORT int Parameterizable::doReadParameter<int>(const std::string& name) const;
template CSAPEX_EXPORT std::string Parameterizable::doReadParameter<std::string>(const std::string& name) const;
template CSAPEX_EXPORT std::pair<int,int> Parameterizable::doReadParameter<std::pair<int,int> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<double,double> Parameterizable::doReadParameter<std::pair<double,double> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<std::string, bool> Parameterizable::doReadParameter<std::pair<std::string, bool> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<double> Parameterizable::doReadParameter<std::vector<double> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<int> Parameterizable::doReadParameter<std::vector<int> >(const std::string& name) const;


template CSAPEX_EXPORT void Parameterizable::doSetParameter<bool>(const std::string& name, const bool& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<double>(const std::string& name, const double& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<int>(const std::string& name, const int& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::string>(const std::string& name, const std::string& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::pair<int,int> > (const std::string& name, const std::pair<int,int>& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::pair<double,double> >(const std::string& name, const std::pair<double,double>& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::pair<std::string, bool> >(const std::string& name, const std::pair<std::string, bool>& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::vector<int> >(const std::string& name, const std::vector<int>& value);
template CSAPEX_EXPORT void Parameterizable::doSetParameter<std::vector<double> >(const std::string& name, const std::vector<double>& value);



void Parameterizable::addParameterCallback(csapex::param::ParameterPtr param, std::function<void(csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);

    param::ParameterWeakPtr pwp = param;
    connections_[param.get()].push_back(param->parameter_changed.connect([this, pwp, cb](csapex::param::Parameter* p) {
        if(param::Parameter::Ptr p = pwp.lock()) {
            parameterChanged(p, cb);
        }
    }));

    if(param->hasState()) {
        parameterChanged(param, cb);
    }
}

void Parameterizable::removeParameterCallbacks(csapex::param::Parameter *param)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    std::unique_lock<std::recursive_mutex > clock(changed_params_mutex_);

    for(auto it = changed_params_.begin(); it != changed_params_.end();) {
        if(param::ParameterPtr p = it->first.lock()) {
            if(p.get() == param) {
                it = changed_params_.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    for(csapex::slim_signal::Connection c : connections_[param]) {
        c.disconnect();
    }
}

void Parameterizable::addParameterCondition(csapex::param::ParameterPtr param, std::function<bool ()> enable_condition)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    csapex::param::ParameterWeakPtr pwp = param;
    conditions_[pwp] = enable_condition;
}
void Parameterizable::addParameterCondition(csapex::param::ParameterPtr param, bool& enable_condition)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    csapex::param::ParameterWeakPtr pwp = param;
    conditions_[pwp] = [&enable_condition](){ return enable_condition; };
}

void Parameterizable::parameterChanged(param::ParameterPtr)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if(!conditions_.empty()) {
        checkConditions(false);
    }
}

void Parameterizable::parameterChanged(csapex::param::ParameterPtr param, std::function<void(csapex::param::Parameter *)> cb)
{
    {
        std::unique_lock<std::recursive_mutex> lock(mutex_);
        std::unique_lock<std::recursive_mutex > clock(changed_params_mutex_);

        for(auto it = changed_params_.begin(); it != changed_params_.end();) {
            param::ParameterWeakPtr pw = it->first;
            param::ParameterPtr p = pw.lock();
            if(p == param) {
                it = changed_params_.erase(it);
            } else {
                ++it;
            }
        }
        changed_params_.push_back(std::make_pair(param, cb));
    }

    if(!silent_) {
        parameters_changed();
    }
}

void Parameterizable::parameterEnabled(csapex::param::Parameter */*param*/, bool /*enabled*/)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    triggerParameterSetChanged();
}



void Parameterizable::checkConditions(bool silent)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    bool change = false;
    setParameterSetSilence(true);
    for(auto it = conditions_.begin(); it != conditions_.end(); ++it) {
        if(csapex::param::ParameterPtr p = it->first.lock()) {
            bool should_be_enabled = it->second();
            if(should_be_enabled != p->isEnabled()) {
                p->setEnabled(should_be_enabled);
                change = true;
            }
        }
    }
    setParameterSetSilence(false);

    if(change && !silent) {
        triggerParameterSetChanged();
    }
}

void Parameterizable::addPersistentParameter(const csapex::param::Parameter::Ptr &param)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->addPersistentParameter(param);
}

void Parameterizable::addHiddenParameter(const csapex::param::Parameter::Ptr &param)
{
    param->setHidden(true);
    addParameter(param);
}

void Parameterizable::addHiddenParameter(const csapex::param::Parameter::Ptr &param, std::function<void(csapex::param::Parameter *)> cb)
{
    param->setHidden(true);
    addParameter(param, cb);
}

void Parameterizable::addTemporaryParameter(const csapex::param::Parameter::Ptr &param)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->addTemporaryParameter(param);
}

void Parameterizable::addTemporaryParameter(const csapex::param::Parameter::Ptr &param, std::function<void (csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->addTemporaryParameter(param);
    addParameterCallback(param, cb);
}

void Parameterizable::removeTemporaryParameter(const csapex::param::Parameter::Ptr &param)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->removeTemporaryParameter(param);
    triggerParameterSetChanged();
}


void Parameterizable::setTemporaryParameters(const std::vector<csapex::param::Parameter::Ptr> &params)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    setParameterSetSilence(true);
    removeTemporaryParameters();
    for(csapex::param::Parameter::Ptr param : params) {
        addTemporaryParameter(param);
    }
    setParameterSetSilence(false);
    triggerParameterSetChanged();
}

void Parameterizable::setTemporaryParameters(const std::vector<csapex::param::Parameter::Ptr> &params, std::function<void (csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    setParameterSetSilence(true);
    removeTemporaryParameters();
    for(csapex::param::Parameter::Ptr param : params) {
        addTemporaryParameter(param, cb);
    }
    setParameterSetSilence(false);
    triggerParameterSetChanged();
}



void Parameterizable::addParameter(const csapex::param::Parameter::Ptr &param)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->addParameter(param);

    param::ParameterWeakPtr pwp = param;
    connections_[param.get()].push_back(param->parameter_changed.connect([this, pwp](csapex::param::Parameter* p) {
        if(param::Parameter::Ptr p = pwp.lock()) {
            parameterChanged(p);
        }
    }));
    connections_[param.get()].push_back(param->parameter_enabled.connect([this](csapex::param::Parameter* p, bool e) { parameterEnabled(p, e); } ));
}

void Parameterizable::addParameter(const csapex::param::Parameter::Ptr &param, std::function<void (csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    addParameter(param);
    addParameterCallback(param, cb);
}


void Parameterizable::addConditionalParameter(const csapex::param::Parameter::Ptr &param, std::function<bool()> enable_condition)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    addParameter(param);
    addParameterCondition(param, enable_condition);
}


void Parameterizable::addConditionalParameter(const csapex::param::Parameter::Ptr &param, std::function<bool()> enable_condition, std::function<void (csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    addParameter(param);
    addParameterCallback(param, cb);
    addParameterCondition(param, enable_condition);
}

void Parameterizable::addConditionalParameter(const csapex::param::Parameter::Ptr &param, bool& condition_variable)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    addParameter(param);
    addParameterCondition(param, condition_variable);
}


void Parameterizable::addConditionalParameter(const csapex::param::Parameter::Ptr &param, bool& condition_variable, std::function<void (csapex::param::Parameter *)> cb)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    addParameter(param);
    addParameterCallback(param, cb);
    addParameterCondition(param, condition_variable);
}


std::vector<csapex::param::Parameter::Ptr> Parameterizable::getParameters() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getParameters();
}

std::vector<csapex::param::Parameter::Ptr> Parameterizable::getTemporaryParameters() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getTemporaryParameters();
}

std::vector<csapex::param::Parameter::Ptr> Parameterizable::getPersistentParameters() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getPersistentParameters();
}

std::size_t Parameterizable::getParameterCount() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getParameterCount();
}

bool Parameterizable::hasParameter(const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->hasParameter(name);
}

csapex::param::Parameter::Ptr Parameterizable::getParameter(const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getParameter(name);
}

csapex::param::Parameter::Ptr Parameterizable::getMappedParameter(const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->getMappedParameter(name);
}

bool Parameterizable::isParameterEnabled(const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return getParameter(name)->isEnabled();
}

void Parameterizable::setParameterEnabled(const std::string &name, bool enabled)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    getParameter(name)->setEnabled(enabled);
}



void Parameterizable::setParameterSetSilence(bool silent)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->setParameterSetSilence(silent);
}

void Parameterizable::removeTemporaryParameters()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    // TODO: handle callbacks!
    for(csapex::param::Parameter::Ptr param : parameter_state_->getTemporaryParameters()) {
        removeParameterCallbacks(param.get());
    }

    parameter_state_->removeTemporaryParameters();
}

void Parameterizable::triggerParameterSetChanged()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    parameter_state_->triggerParameterSetChanged();
}

Parameterizable::ChangedParameterList Parameterizable::getChangedParameters()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    ChangedParameterList changed_params;

    std::unique_lock<std::recursive_mutex > clock(changed_params_mutex_);

    if(!param_updates_.empty()) {
//        setSilent(true);
        while(!param_updates_.empty()) {
            for(auto& entry : param_updates_) {
                entry.second();
            }
            param_updates_.clear();
        }
//        setSilent(false);
    }

    changed_params = changed_params_;
    changed_params_.clear();

    return changed_params;
}


GenericState::Ptr Parameterizable::getParameterStateClone() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_->clone();
}

GenericState::Ptr Parameterizable::getParameterState()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return parameter_state_;
}

void Parameterizable::setParameterState(Memento::Ptr memento)
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState> (memento);
    apex_assert_hard(m.get());

    parameter_state_->setFrom(*m);
}
