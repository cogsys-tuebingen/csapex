#ifndef CLONABLE_H
#define CLONABLE_H

/// PROJECT
#include <csapex/utility/data_traits.hpp>
#include <csapex/utility/type.h>
#include <csapex/utility/assert.h>

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

    virtual bool cloneDataFrom(const Clonable& other) = 0;
    virtual std::shared_ptr<Clonable> makeEmptyInstance() const = 0;

    virtual bool hasData(const std::type_info& type) const
    {
        return false;
    }
    template <typename T>
    const T* getDataPtr() const
    {
        const std::type_info& type = typeid(T);
        apex_assert_hard(hasData(type));
        auto ptr = getDataPtrUnsafe(type);
        if (ptr == nullptr) {
            throw std::logic_error(std::string("access to data of type ") + type2name(type) + std::string(" is not possible"));
        }
        return static_cast<const T*>(ptr);
    }

protected:
    virtual const void* getDataPtrUnsafe(const std::type_info& type) const
    {
        throw std::logic_error(std::string("access to data of type ") + type2name(type) + std::string(" is not implemented"));
    }
};

}  // namespace csapex

#define _CLONABLE_BLOCK_CLONE(Instance)                                                                                                                                                                \
public:                                                                                                                                                                                                \
    std::shared_ptr<Instance> clone() const                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        return cloneAs<Instance>();                                                                                                                                                                    \
    }
#define _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance)                                                                                                                                                   \
protected:                                                                                                                                                                                             \
    std::shared_ptr<Clonable> makeEmptyInstance() const override                                                                                                                                       \
    {                                                                                                                                                                                                  \
        return std::shared_ptr<Instance>(new Instance);                                                                                                                                                \
    }
#define _CLONABLE_BLOCK_MAKE_EMPTY(Instance, ...)                                                                                                                                                      \
protected:                                                                                                                                                                                             \
    std::shared_ptr<Clonable> makeEmptyInstance() const override                                                                                                                                       \
    {                                                                                                                                                                                                  \
        return std::shared_ptr<Instance>(new Instance(__VA_ARGS__));                                                                                                                                   \
    }
#define _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)                                                                                                                                                      \
public:                                                                                                                                                                                                \
    bool cloneDataFrom(const Clonable& other) override                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        return cloneData(dynamic_cast<const Instance&>(other));                                                                                                                                        \
    }
#define _CLONABLE_BLOCK_CLONE_DATA(Instance)                                                                                                                                                           \
protected:                                                                                                                                                                                             \
    virtual bool cloneData(const Instance& other)                                                                                                                                                      \
    {                                                                                                                                                                                                  \
        *this = other;                                                                                                                                                                                 \
        return true;                                                                                                                                                                                   \
    }

#define CLONABLE_IMPLEMENTATION_NO_CLONE_DATA_FROM(Instance)                                                                                                                                           \
    _CLONABLE_BLOCK_CLONE_DATA(Instance)                                                                                                                                                               \
    _CLONABLE_BLOCK_CLONE(Instance)                                                                                                                                                                    \
    _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance)

#define CLONABLE_IMPLEMENTATION(Instance)                                                                                                                                                              \
    CLONABLE_IMPLEMENTATION_NO_CLONE_DATA_FROM(Instance)                                                                                                                                               \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(Instance)                                                                                                                                                \
    _CLONABLE_BLOCK_CLONE(Instance)                                                                                                                                                                    \
    _CLONABLE_BLOCK_MAKE_DEFAULT_EMPTY(Instance)                                                                                                                                                       \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_CONSTRUCTOR(Instance, ...)                                                                                                                                             \
    _CLONABLE_BLOCK_CLONE_DATA(Instance)                                                                                                                                                               \
    _CLONABLE_BLOCK_CLONE(Instance)                                                                                                                                                                    \
    _CLONABLE_BLOCK_MAKE_EMPTY(Instance, __VA_ARGS__)                                                                                                                                                  \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#define CLONABLE_IMPLEMENTATION_NO_DEFAULT_CONSTRUCTOR(Instance)                                                                                                                                       \
    _CLONABLE_BLOCK_CLONE_DATA(Instance)                                                                                                                                                               \
    _CLONABLE_BLOCK_CLONE(Instance)                                                                                                                                                                    \
    _CLONABLE_BLOCK_CLONE_DATA_FROM(Instance)

#endif  // CLONABLE_H
