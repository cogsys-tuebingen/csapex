#ifndef MEMENTO_H
#define MEMENTO_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

class Memento
{
public:
    typedef boost::shared_ptr<Memento> Ptr;

public:
    Memento();
    virtual ~Memento();
};

#endif // MEMENTO_H
