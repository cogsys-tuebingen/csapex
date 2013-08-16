/// HEADER
#include <csapex/memento.h>

using namespace csapex;

const Memento::Ptr Memento::NullPtr;

Memento::Memento()
{
}

Memento::~Memento()
{
}

void Memento::writeYaml(YAML::Emitter&) const
{
}

void Memento::readYaml(const YAML::Node&)
{
}
