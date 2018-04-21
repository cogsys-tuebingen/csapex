/// HEADER
#include <csapex/param/set_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(SetParameter)

using namespace csapex;
using namespace param;

SetParameter::SetParameter()
    : ParameterImplementation("noname", ParameterDescription()), txt_("")
{
}


SetParameter::SetParameter(const std::string &name, const ParameterDescription &description)
    : ParameterImplementation(name, description), txt_("")
{
}

SetParameter::~SetParameter()
{

}

void SetParameter::setSet(const std::vector<std::string> &set)
{
    set_.clear();
    bool serialized_found = txt_.empty();
    for(typename std::vector<std::string>::const_iterator it = set.begin(); it != set.end(); ++it) {
        set_[*it] = *it;
        if(!serialized_found && txt_ == *it) {
            serialized_found = true;
        }
    }
    if(!serialized_found) {
        set_[txt_] = txt_;
    }

    if(!set.empty()) {
        if(def_.empty()) {
            def_ = set.front();
        }

        if(value_.empty()) {
            value_ = def_;
        }
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

std::vector<std::string> SetParameter::getSetTexts() const
{
    std::vector<std::string> res;
    for(const auto& pair : set_) {
        res.push_back(pair.first);
    }
    return res;
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

    throw std::runtime_error("cannot get the text for parameter '" + name() + "'");
}

std::string SetParameter::defText() const
{
    return convertToString(def_);
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

void SetParameter::get_unsafe(boost::any& out) const
{
    if(value_.empty()) {
        out = def_;
    } else {
        out = value_;
    }
}


bool SetParameter::set_unsafe(const boost::any &v)
{
    if(v.type() == typeid(std::pair<std::string, bool>)) {
        auto pair = boost::any_cast<std::pair<std::string, bool>>(v);
        setByName(pair.first);
        return true;
    }


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


void SetParameter::cloneDataFrom(const Clonable &other)
{
    if(const SetParameter* set = dynamic_cast<const SetParameter*>(&other)) {
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
            *this = *set;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void SetParameter::doSerialize(YAML::Node& n) const
{
    n["txt"] = getText();

    if(value_.type() == typeid(int)) {
        doSerializeImplementation<int>("int", n);

    } else if(value_.type() == typeid(double)) {
        doSerializeImplementation<double>("double", n);

    } else if(value_.type() == typeid(bool)) {
        doSerializeImplementation<bool>("bool", n);

    } else if(value_.type() == typeid(std::string)) {
        doSerializeImplementation<std::string>("string", n);
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
    if(n["txt"].IsDefined()) {
        txt_ = n["txt"].as<std::string>();
    } else {
        // backward compatibility
        txt_ = "unknown";
    }

    if(n["int"].IsDefined()) {
        doDeserializeImplementation<int>("int", n);

    } else if(n["double"].IsDefined()) {
        doDeserializeImplementation<double>("double", n);

    } else if(n["bool"].IsDefined()) {
        doDeserializeImplementation<bool>("bool", n);

    } else if(n["string"].IsDefined()) {
        doDeserializeImplementation<std::string>("string", n);
    }
}


template <typename T>
void SetParameter::doSerializeImplementation(const std::string& type_name, YAML::Node& n) const
{
    n["txt"] = getText();

    n[type_name] = boost::any_cast<T>(value_);

    std::vector<std::pair<std::string, T>> values;
    for(const std::pair<std::string, boost::any>& pair : set_) {
        values.push_back(std::make_pair(pair.first, boost::any_cast<T>(pair.second)));
    }

    n["values"] = values;
}

template <typename T>
void SetParameter::doDeserializeImplementation(const std::string& type_name, const YAML::Node& n)
{
    T value = __read<T>(n[type_name]);
    value_ = value;

    if(n["values"].IsDefined()) {
        auto values = n["values"].as<std::vector<std::pair<std::string, T>>>();

        bool found = false;
        for(const std::pair<std::string, T>& v : values) {
            set_[v.first] = v.second;
            if(v.second == value) {
                found = true;
            }
        }

        if(!found) {
            set_[name_] = value_;
        }

    } else {
        set_[name_] = value_;
    }

}

bool SetParameter::accepts(const std::type_info& type) const
{
    return type == value_.type() || type == typeid(std::pair<std::string, bool>);
}



void SetParameter::serialize(SerializationBuffer &data) const
{
    Parameter::serialize(data);

    data << value_;
    data << txt_;
    data << set_;
    data << def_;
}

void SetParameter::deserialize(const SerializationBuffer& data)
{
    Parameter::deserialize(data);

    data >> value_;
    data >> txt_;
    data >> set_;
    data >> def_;
}
