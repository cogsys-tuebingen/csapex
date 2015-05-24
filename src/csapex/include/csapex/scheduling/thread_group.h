#ifndef THREAD_GROUP_H
#define THREAD_GROUP_H

/// SYSTEM
#include <string>

namespace csapex
{

struct ThreadGroup
{
    ThreadGroup(int id, std::string name)
        : id(id), name(name)
    {}

    int id;
    std::string name;
};

}

#endif // THREAD_GROUP_H

