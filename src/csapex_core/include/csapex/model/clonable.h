#ifndef CLONABLE_H
#define CLONABLE_H

/// PROJECT
#include <csapex/utility/data_traits.hpp>

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
        auto res = makeEmptyInstance();
        res->cloneDataFrom(*this);
        return res;
    }

    template <typename Implementation>
    std::shared_ptr<Implementation> cloneAs() const
    {
        return std::dynamic_pointer_cast<Implementation>(cloneRaw());
    }

    virtual void cloneDataFrom(const Clonable& other) = 0;
    virtual std::shared_ptr<Clonable> makeEmptyInstance() const = 0;
};

}

#define _CLONABLE_BLOCK_CLONE(Instance) \
public: \
    std::shared_ptr<Instance> clone() const { \
        return cloneAs<Instance>(); \
    }
#define _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance) \
protected: \
    std::shared_ptr<Clonable> makeEmptyInstance() const override { \
        return std::shared_ptr<Instance>(new Instance); \
    }
#define _CLONABLE_BLOCK_MAKE_EMPTY(Instance, ...) \
protected: \
    std::shared_ptr<Clonable> makeEmptyInstance() const override { \
        return std::shared_ptr<Instance>(new Instance(__VA_ARGS__)); \
    }
#define _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance) \
protected: \
    void cloneDataFrom(const Clonable &other) override { \
        cloneData(dynamic_cast<const Instance&>(other)); \
    }
#define _CLONABLE_BLOCK_CLONE_DATA(Instance) \
protected: \
    virtual void cloneData(const Instance &other) { \
        *this = other; \
    } \

#define CLONABLE_IMPLEMENTATION(Instance) \
    _CLONABLE_BLOCK_CLONE_DATA(Instance) \
    _CLONABLE_BLOCK_CLONE(Instance) \
    _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance) \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(Instance) \
    _CLONABLE_BLOCK_CLONE(Instance) \
    _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance) \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_CONSTRUCTOR(Instance, ...) \
    _CLONABLE_BLOCK_CLONE_DATA(Instance) \
    _CLONABLE_BLOCK_CLONE(Instance) \
    _CLONABLE_BLOCK_MAKE_EMPTY(Instance, __VA_ARGS__) \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_NO_DEFAULT_CONSTRUCTOR(Instance) \
    _CLONABLE_BLOCK_CLONE_DATA(Instance) \
    _CLONABLE_BLOCK_CLONE(Instance) \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#endif // CLONABLE_H
