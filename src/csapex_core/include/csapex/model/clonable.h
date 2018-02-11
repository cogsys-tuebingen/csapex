#ifndef CLONABLE_H
#define CLONABLE_H

/// SYSTEM
#include <memory>

namespace csapex
{

class Clonable
{
public:
    virtual ~Clonable() = default;
    virtual std::shared_ptr<Clonable> cloneRaw() const
    {
        auto res = makeEmptyClone();
        *res = *this;
        return res;
    }

    template <typename Implementation>
    std::shared_ptr<Implementation> clone() const
    {
        return std::dynamic_pointer_cast<Implementation>(cloneRaw());
    }

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const = 0;
};

}

#endif // CLONABLE_H
