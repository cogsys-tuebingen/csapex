/// HEADER
#include <csapex/param/set_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace param;

SetParameter::SetParameter()
    : Parameter("noname", ParameterDescription()), txt_("")
{
}


SetParameter::SetParameter(const std::string &name, const ParameterDescription &description)
    : Parameter(name, description), txt_("")
{
}

SetParameter::~SetParameter()
{

}

void SetParameter::setSet(const std::vector<std::string> &set)
{
    set_.clear();
    bool found = txt_.empty();
    for(typename std::vector<std::string>::const_iterator it = set.begin(); it != set.end(); ++it) {
        set_[*it] = *it;
        if(!found && txt_ == *it) {
            found = true;
        }
    }
    if(!found) {
        set_[txt_] = value_;
    }
    scope_changed(this);
}

void SetParameter::setByName(const std::string &name)
{
    std::map<std::string, boost::any>::iterator pos = set_.find(name);
    if(pos == set_.end()) {
        throw std::runtime_error(std::string("no such parameter: ") + name);
    }

    value_ = pos->second;
    txt_ = getText();
    triggerChange();
}

int SetParameter::noParameters() const
{
    return set_.size();
}

std::string SetParameter::convertToString(const boost::any &v) const
{
    if(v.type() == typeid(std::string)) {
        return boost::any_cast<std::string> (v);
    }

    std::stringstream ss;
    if(v.type() == typeid(int)) {
        ss << boost::any_cast<int> (v);

    } else if(v.type() == typeid(double)) {
        ss << boost::any_cast<double> (v);

    } else if(v.type() == typeid(bool)) {
        ss << boost::any_cast<bool> (v);

    } else {
        throw std::runtime_error(std::string("unsupported type: ") + v.type().name());
    }

    return ss.str();
}

std::string SetParameter::getText(int idx) const
{
    std::map<std::string, boost::any>::const_iterator i = set_.begin();
    std::advance(i, idx);
    return i->first;
}

std::string SetParameter::getText() const
{
    std::string str =  convertToString(value_);
    for(std::map<std::string, boost::any>::const_iterator it = set_.begin(); it != set_.end(); ++it) {
        if(convertToString(it->second) == str) {
            return it->first;
        }
    }

    if(is<std::string>()) {
        return as<std::string>();
    }

    std::abort();
    throw std::runtime_error("cannot get the text for parameter '" + name() + "'");
}


const std::type_info& SetParameter::type() const
{
    Lock l = lock();
    return def_.type();
}

std::string SetParameter::toStringImpl() const
{
    return std::string("[set: ") + convertToString(value_)  + "]";
}

boost::any SetParameter::get_unsafe() const
{
    if(value_.empty()) {
        return def_;
    } else {
        return value_;
    }
}


bool SetParameter::set_unsafe(const boost::any &v)
{
    bool change = true;
    if(!value_.empty()) {
        if(v.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(v);
        } else if(v.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(v);
        } else if(v.type() == typeid(bool)) {
            change = boost::any_cast<bool>(value_) != boost::any_cast<bool>(v);
        } else if(v.type() == typeid(std::string)) {
            change = boost::any_cast<std::string>(value_) != boost::any_cast<std::string>(v);
        }
    }

    if(change) {
        value_ = v;
        txt_ = getText();

        return true;
    }

    return false;
}


void SetParameter::doSetValueFrom(const Parameter &other)
{
    const SetParameter* set = dynamic_cast<const SetParameter*>(&other);
    if(set) {
        txt_ = set->txt_;
        bool change = false;
        if(set_.find(txt_) == set_.end()) {
            set_[txt_] = set->value_;
            change = true;
        }
        if(value_.type() == typeid(int)) {
            change = boost::any_cast<int>(value_) != boost::any_cast<int>(set->value_);
        } else if(value_.type() == typeid(double)) {
            change = boost::any_cast<double>(value_) != boost::any_cast<double>(set->value_);
        } else if(value_.type() == typeid(bool)) {
            change = boost::any_cast<bool>(value_) != boost::any_cast<bool>(set->value_);
        } else if(value_.type() == typeid(std::string)) {
            change = boost::any_cast<std::string>(value_) != boost::any_cast<std::string>(set->value_);
        }
        if(change) {
            value_ = set->value_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}


void SetParameter::doClone(const Parameter &other)
{
    const SetParameter* set = dynamic_cast<const SetParameter*>(&other);
    if(set) {
        value_ = set->value_;
        txt_ = set->txt_;
        set_ = set->set_;
        def_ = set->def_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void SetParameter::doSerialize(YAML::Node& n) const
{
    n["txt"] = getText();

    if(value_.type() == typeid(int)) {
        n["int"] = boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        n["double"] = boost::any_cast<double> (value_);

    } else if(value_.type() == typeid(bool)) {
        n["bool"] = boost::any_cast<bool> (value_);

    } else if(value_.type() == typeid(std::string)) {
        n["string"] = boost::any_cast<std::string> (value_);
    }
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    return n.as<T>();
}
}

void SetParameter::doDeserialize(const YAML::Node& n)
{
    if(!n["name"].IsDefined()) {
        return;
    }

    name_ = n["name"].as<std::string>();

    if(n["txt"].IsDefined()) {
        txt_ = n["txt"].as<std::string>();
    } else {
        // backward compability
        txt_ = "unknown";
    }

    if(n["int"].IsDefined()) {
        value_ = __read<int>(n["int"]);

    } else if(n["double"].IsDefined()) {
        value_ = __read<double>(n["double"]);

    } else if(n["bool"].IsDefined()) {
        value_ = __read<bool>(n["bool"]);

    } else if(n["string"].IsDefined()) {
        value_ = __read<std::string>(n["string"]);
    }
    set_[name_] = value_;
}
