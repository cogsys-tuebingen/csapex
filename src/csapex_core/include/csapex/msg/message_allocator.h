#ifndef MESSAGE_ALLOCATOR_H
#define MESSAGE_ALLOCATOR_H

/// PROJECT
#include <csapex_core/csapex_core_export.h>

/// SYSTEM
#include <memory>

namespace csapex
{
class MessageAllocatorImplementationInterface
{
public:
    class Deleter
    {
    public:
        Deleter(MessageAllocatorImplementationInterface* alloc) : alloc_(alloc)
        {
        }

        template <typename T>
        void operator()(T* ptr) noexcept
        {
            alloc_->deallocate(reinterpret_cast<uint8_t*>(ptr));
        }

    private:
        MessageAllocatorImplementationInterface* alloc_;
    };

public:
    virtual ~MessageAllocatorImplementationInterface() = default;

    virtual uint8_t* allocate() = 0;
    virtual void deallocate(uint8_t* data) = 0;
};

namespace detail
{
template <typename T>
static uint8_t* getPointer(T* ptr)
{
    return reinterpret_cast<uint8_t*>(ptr);
}
template <typename Wrapper>
static uint8_t* getPointer(const Wrapper& ptr)
{
    return ptr.get();
}
}  // namespace detail

template <typename T, typename Alloc>
class MessageAllocatorImplementation : public MessageAllocatorImplementationInterface
{
public:
    MessageAllocatorImplementation(const Alloc& alloc) : alloc_(alloc)
    {
    }

    uint8_t* allocate() override
    {
        return reinterpret_cast<uint8_t*>(detail::getPointer(alloc_.allocate(sizeof(T))));
    }

    void deallocate(uint8_t* data) override
    {
        alloc_.deallocate(data, sizeof(T));
    }

private:
    Alloc alloc_;
};

class CSAPEX_CORE_EXPORT MessageAllocator
{
public:
    MessageAllocator();
    virtual ~MessageAllocator();

    template <typename T, typename... Args>
    std::shared_ptr<T> allocate(Args&&... args)
    {
        if (allocator_) {
            uint8_t* raw = allocator_->allocate();
            try {
                T* data = new (raw) T(std::forward<Args>(args)...);
                std::shared_ptr<T> res(data, MessageAllocatorImplementationInterface::Deleter(allocator_));
                return res;

            } catch (...) {
                // if anything happens, deallocate
                allocator_->deallocate(raw);
                return nullptr;
            }
        } else {
            return std::make_shared<T>(std::forward<Args>(args)...);
        }
    }

    template <typename T, typename Alloc>
    void setAllocator(const Alloc& alloc)
    {
        delete allocator_;
        allocator_ = new MessageAllocatorImplementation<T, Alloc>(alloc);
    }

private:
    MessageAllocatorImplementationInterface* allocator_;
};

}  // namespace csapex

#endif  // MESSAGE_ALLOCATOR_H
