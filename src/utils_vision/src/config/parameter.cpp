/// HEADER
#include "parameter.h"

///// SYSTEM
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>

using namespace vision;

Parameter::Parameter()
{
}

Parameter::Parameter(const std::string &name)
    : name_(name)
{
}

void Parameter::setFrom(const Parameter &other)
{
    value_ = other.value_;
}

std::string Parameter::name() const
{
    return name_;
}

namespace {
class serialization_visitor : public boost::static_visitor<void>
{
public:
    serialization_visitor(YAML::Emitter& e)
        : e(e)
    {}

    void operator () (int i) const
    {
        e << YAML::Key << "int" << YAML::Value << i;
    }
    void operator () (double d) const
    {
        e << YAML::Key << "double" << YAML::Value << d;
    }
    void operator () (bool b) const
    {
        e << YAML::Key << "bool" << YAML::Value << b;
    }
    void operator () (const std::string& s) const
    {
        e << YAML::Key << "string" << YAML::Value << s;
    }
private:
    YAML::Emitter& e;
};
}


void Parameter::write(YAML::Emitter& e) const
{
    e << YAML::BeginMap;
    e << YAML::Key << "name" << YAML::Value << name();

    serialization_visitor sv(e);
    boost::apply_visitor(sv, value_);

    e << YAML::EndMap;
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void Parameter::read(const YAML::Node& n)
{
    if(!n.FindValue("name")) {
        return;
    }

    n["name"] >> name_;

    if(n.FindValue("int")) {
        value_ = __read<int>(n["int"]);

    } else if(n.FindValue("double")) {
        value_ = __read<double>(n["double"]);

    } else if(n.FindValue("bool")) {
        value_ = __read<bool>(n["bool"]);

    } else if(n.FindValue("string")) {
        value_ = __read<std::string>(n["string"]);
    }
}

//void Parameter::write(YAML::Emitter& e) const
//{
//    std::stringstream ss;
//    boost::archive::text_oarchive oa(ss);

//    oa & value_;

//    e << YAML::BeginMap;
//    e << YAML::Key << "data" << YAML::Value << ss.str();
//    e << YAML::EndMap;
//}

//void Parameter::read(const YAML::Node& n)
//{
//    if(!n.FindValue("data")) {
//        return;
//    }

//    std::string data;
//    n["data"] >> data;

//    std::stringstream ss(data);
//    boost::archive::text_iarchive ia(ss);

//    ia & value_;
//}
