/// HEADER
#include <utils_param/bitset_parameter.h>

using namespace param;

BitSetParameter::BitSetParameter()
    : Parameter("noname")
{
}


BitSetParameter::BitSetParameter(const std::string &name)
    : Parameter(name)
{
}

BitSetParameter::~BitSetParameter()
{

}

void BitSetParameter::setByName(const std::string &name)
{
    for(std::vector< std::pair<std::string, int> >::iterator it = set_.begin(); it != set_.end(); ++it) {
        if(it->first == name) {
            value_ = it->second;
            parameter_changed(this);
            return;
        }
    }

    throw std::runtime_error(std::string("no such parameter: ") + name);
}

void BitSetParameter::setBitSet(const std::vector< std::pair<std::string, int> >& set) {
    set_ = set;
}

void BitSetParameter::clear()
{
    value_ = 0;
    parameter_changed(this);
}

void BitSetParameter::setBits(const std::vector<std::string> &elements, bool silent)
{
    bool change = false;

    for(std::vector< std::pair<std::string, int> >::iterator set_it = set_.begin(); set_it != set_.end(); ++set_it) {
        bool found = false;
        const std::string& e = set_it->first;
        for(std::vector<std::string>::const_iterator e_it = elements.begin(); e_it != elements.end(); ++e_it) {
            if(e == *e_it) {
                found = true;
                break;
            }
        }
        if(found) {
            if(!isSet(e)) {
                setBit(e, true);
                change = true;
            }
        } else {
            if(isSet(e)) {
                clearBit(e, true);
                change = true;
            }
        }
    }

    if(change && !silent) {
        parameter_changed(this);
    }
}

void BitSetParameter::setBitTo(const std::string &element, bool set, bool silent)
{
    for(std::vector< std::pair<std::string, int> >::iterator it = set_.begin(); it != set_.end(); ++it) {
        if(it->first == element) {
            if(set) {
                value_ |= it->second;
            } else {
                value_ &= ~(it->second);
            }

            if(!silent) {
                parameter_changed(this);
            }
            return;
        }
    }
}

void BitSetParameter::setBit(const std::string &element, bool silent)
{
    setBitTo(element, true, silent);
}

void BitSetParameter::clearBit(const std::string &element, bool silent)
{
    setBitTo(element, false, silent);
}

bool BitSetParameter::isSet(const std::string &element) const
{
    for(std::vector< std::pair<std::string, int> >::const_iterator it = set_.begin(); it != set_.end(); ++it) {
        if(it->first == element) {
            int target = it->second;

            return (value_ & target) == target;
        }
    }
    return false;
}

int BitSetParameter::noParameters() const
{
    return set_.size();
}

std::string BitSetParameter::getName(int idx) const
{
    return set_[idx].first;
}

std::string BitSetParameter::getName() const
{
    throw std::runtime_error("cannot get the name for parameter '" + name() + "'");
}


const std::type_info& BitSetParameter::type() const
{
    return typeid(int);
}

std::string BitSetParameter::toStringImpl() const
{
    return std::string("[bitset: ") +  "]";
}

boost::any BitSetParameter::get_unsafe() const
{
    return value_;
}


void BitSetParameter::set_unsafe(const boost::any &v)
{
    value_ = boost::any_cast<int>(v);
}


void BitSetParameter::setFrom(const Parameter &other)
{
    const BitSetParameter* range = dynamic_cast<const BitSetParameter*>(&other);
    if(range) {
        value_ = range->value_;
        parameter_changed(this);
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void BitSetParameter::write(YAML::Emitter& e) const
{
    e << YAML::BeginMap;
    e << YAML::Key << "name" << YAML::Value << name();
    e << YAML::Key << "type" << YAML::Value << "bitset";
    e << YAML::Key << "int" << YAML::Value << boost::any_cast<int> (value_);
    e << YAML::EndMap;
}

void BitSetParameter::read(const YAML::Node& n)
{
    if(!n.FindValue("name")) {
        return;
    }

    n["name"] >> name_;
    if(n.FindValue("int")) {
        n["int"] >> value_;
    }
}
