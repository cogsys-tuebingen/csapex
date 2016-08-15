/// PROJECT
#include <csapex/factory/generic_node.hpp>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/token_traits.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <boost/function_types/parameter_types.hpp>
#include <boost/type_traits.hpp>
#include <boost/mpl/transform.hpp>

namespace csapex {

class CSAPEX_EXPORT GenericNodeFactory
{
public:
    /**
     * @brief wrapFunction creates a Node instance from a function pointer
     *        Let T be a Message type, then
     *          const T&   gets converted into an Input of type T
     *                T&   gets converted into an Output of type T
     *        Other types are converted to parameters
     * @param f is a pointer to the function that should be converted
     * @return an instance of the wrapped Node
     */
    template<typename F, typename Info = generic_node::DefaultInfo>
    static Node::Ptr wrapFunction(F f)
    {
        typedef typename boost::function_types::parameter_types<F>::type params;
        return std::make_shared<GenericNode<params, Info>>(f);
    }


    /**
     * @brief createConstructorFromFunction creates a NodeConstructor instance from a function pointer
     * @param f is a pointer to the function that should be converted
     * @param name is the name of the wrapped function
     * @param description is a description of the wrapped function
     * @param settings
     * @param tags
     * @param icon
     * @return the generated NodeConstructor
     */
    template<typename Info, typename F>
    static NodeConstructor::Ptr createConstructorFromFunction(F f, const std::string& name)
    {
        return std::make_shared<csapex::NodeConstructor>(name, std::bind(&GenericNodeFactory::wrapFunction<F, Info>, f));
    }
    template<typename F>
    static NodeConstructor::Ptr createConstructorFromFunction(F f, const std::string& name)
    {
        return createConstructorFromFunction<generic_node::DefaultInfo, F>(f, name);
    }
};

}
