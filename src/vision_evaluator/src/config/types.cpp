/// HEADER
#include "types.h"

/// SYSTEM
#include <algorithm>
#include <iostream>
#include <string>
#include <typeinfo>

using namespace Types;

template <class T>
typename Parameter<T>::SubType Parameter<T>::read(const std::string& type)
{
    return instance().readName(type);
}
template <class T>
std::string Parameter<T>::write(SubType type)
{
    return instance().writeName(type);
}
template <class T>
std::string Parameter<T>::write(int type)
{
    return instance().writeName(type);
}

template <class T>
typename Parameter<T>::SubType Parameter<T>::readName(const std::string& type)
{
    std::string type_upper = type;
    std::transform(type_upper.begin(), type_upper.end(), type_upper.begin(), ::toupper);

    for(int i = T::FIRST; i != T::COUNT; ++i) {
        if(map[static_cast<SubType>(i)] == type_upper) {
            return (SubType) i;
        }
    }

    ERROR(typeid(T).name() << " does not recognize Parameter " << type_upper << ". Check spelling!");
    throw IllegalException();
}

void DescriptorImp::initialize(std::map<ID, std::string>& map)
{
    map[BRISK] = "BRISK";
    map[BRIEF] = "BRIEF";
    map[SURF] = "SURF";
    map[SIFT] = "SIFT";
    map[ORB] = "ORB";
    map[FREAK] = "FREAK";
}

void KeypointImp::initialize(std::map<ID, std::string> &map)
{
    map[FAST] = "FAST";
    map[AGAST] = "AGAST";
    map[BRISK] = "BRISK";
    map[SURF] = "SURF";
    map[SIFT] = "SIFT";
    map[ORB] = "ORB";
    map[MSER] = "MSER";
    map[STAR] = "STAR";
    map[GFTT] = "GFTT";
}

void StrategyImp::initialize(std::map<ID, std::string> &map)
{
    map[NAIVE] = "FAST";
    map[BIN] = "BIN";
    map[BAG] = "BAG";
    map[BOW] = "BOW";
}


/// INSTANCIATION

template class Parameter<KeypointImp>;
template class Parameter<DescriptorImp>;
template class Parameter<StrategyImp>;
