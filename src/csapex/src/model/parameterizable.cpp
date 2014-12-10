/// HEADER
#include <csapex/model/parameterizable.h>

/// PROJECT
#include <csapex/model/generic_state.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QtGlobal>

using namespace csapex;


Parameterizable::Parameterizable()
    : changed_params_mutex_(new boost::mutex), parameter_state_(new GenericState)
{

}

Parameterizable::~Parameterizable()
{
    delete changed_params_mutex_;
}

void Parameterizable::addParameterCallback(param::Parameter* param, boost::function<void(param::Parameter *)> cb)
{
    connections_[param].push_back(param->parameter_changed.connect(boost::bind(&Parameterizable::parameterChanged, this, _1, cb)));
    if(param->hasState()) {
        parameterChanged(param, cb);
    }
}

void Parameterizable::removeParameterCallbacks(param::Parameter *param)
{
    boost::lock_guard<boost::mutex> lock(*changed_params_mutex_);

    typedef std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > PAIR;
    for(std::vector<PAIR>::iterator it = changed_params_.begin(); it != changed_params_.end();) {
        const PAIR& p = *it;
        if(p.first == param) {
            it = changed_params_.erase(it);
        } else {
            ++it;
        }
    }

    foreach(boost::signals2::connection c, connections_[param]) {
        c.disconnect();
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
    boost::lock_guard<boost::mutex> lock(*changed_params_mutex_);
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

void Parameterizable::addPersistentParameter(const param::Parameter::Ptr &param)
{
    parameter_state_->addPersistentParameter(param);
}

void Parameterizable::addTemporaryParameter(const param::Parameter::Ptr &param)
{
    parameter_state_->addTemporaryParameter(param);
}

void Parameterizable::addTemporaryParameter(const param::Parameter::Ptr &param, boost::function<void (param::Parameter *)> cb)
{
    parameter_state_->addTemporaryParameter(param);
    addParameterCallback(param.get(), cb);
}

void Parameterizable::setTemporaryParameters(const std::vector<param::Parameter::Ptr> &params)
{
    setParameterSetSilence(true);
    removeTemporaryParameters();
    Q_FOREACH(param::Parameter::Ptr param, params) {
        addTemporaryParameter(param);
    }
    setParameterSetSilence(false);
    triggerParameterSetChanged();
}

void Parameterizable::setTemporaryParameters(const std::vector<param::Parameter::Ptr> &params, boost::function<void (param::Parameter *)> cb)
{
    setParameterSetSilence(true);
    removeTemporaryParameters();
    Q_FOREACH(param::Parameter::Ptr param, params) {
        addTemporaryParameter(param, cb);
    }
    setParameterSetSilence(false);
    triggerParameterSetChanged();
}



void Parameterizable::addParameter(const param::Parameter::Ptr &param)
{
    parameter_state_->addParameter(param);

    connections_[param.get()].push_back(param->parameter_changed.connect(boost::bind(&Parameterizable::parameterChanged, this, _1)));
    connections_[param.get()].push_back(param->parameter_enabled.connect(boost::bind(&Parameterizable::parameterEnabled, this, _1, _2)));
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
    return parameter_state_->getParameters();
}

std::size_t Parameterizable::getParameterCount() const
{
    return parameter_state_->getParameterCount();
}

param::Parameter::Ptr Parameterizable::getParameter(const std::string &name) const
{
    return parameter_state_->getParameter(name);
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
    parameter_state_->setParameterSetSilence(silent);
}

void Parameterizable::removeTemporaryParameters()
{
    // TODO: handle callbacks!
    foreach(param::Parameter::Ptr param, parameter_state_->getTemporaryParameters()) {
        removeParameterCallbacks(param.get());
    }

    parameter_state_->removeTemporaryParameters();
}

void Parameterizable::triggerParameterSetChanged()
{
    parameter_state_->triggerParameterSetChanged();
}

Parameterizable::ChangedParameterList Parameterizable::getChangedParameters()
{
    ChangedParameterList changed_params;

    boost::lock_guard<boost::mutex> lock(*changed_params_mutex_);
    changed_params = changed_params_;
    changed_params_.clear();

    return changed_params;
}


GenericState::Ptr Parameterizable::getParameterStateClone() const
{
    return parameter_state_->clone();
}

GenericState::Ptr Parameterizable::getParameterState()
{
    return parameter_state_;
}

void Parameterizable::setParameterState(Memento::Ptr memento)
{
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (memento);
    apex_assert_hard(m.get());

    parameter_state_->setFrom(*m);

    // TODO: is this necessary?
//    triggerModelChanged();
}
