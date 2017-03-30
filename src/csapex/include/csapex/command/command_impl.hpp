#ifndef COMMAND_IMPL_HPP
#define COMMAND_IMPL_HPP

/// PROJECT
#include <csapex/command/command.h>
#include <csapex/utility/type.h>

namespace csapex
{

template <typename I>
class CommandImplementation : public Command
{
protected:
    CommandImplementation(const AUUID& graph_uuid)
        : Command(graph_uuid)
    {

    }

    CommandImplementation()
    {
    }

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new I);
    }

    std::string getType() const override
    {
        return typeName();
    }

protected:
    void cloneFrom(const Command& other) override
    {
        const I* instance = dynamic_cast<const I*>(&other);
        if(instance) {
            *this = *instance;
        }
    }

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }
};

}

#define COMMAND_HEADER_NO_DEFAULT(Instance)\
protected: \
    friend class CommandImplementation<Instance>

#define COMMAND_HEADER(Instance)\
public: \
    Instance() {} \
protected: \
    COMMAND_HEADER_NO_DEFAULT(Instance)

#endif // COMMAND_IMPL_HPP
