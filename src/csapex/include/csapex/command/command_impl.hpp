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
public:
    void serialize(SerializationBuffer &/*data*/) const override
    {
        // by default, no need for serialization
    }

    void deserialize(SerializationBuffer& /*data*/) override
    {
        // by default, no need for serialization
    }

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
protected: \
    Instance() {} \
    COMMAND_HEADER_NO_DEFAULT(Instance)

#endif // COMMAND_IMPL_HPP
