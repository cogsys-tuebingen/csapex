/// PROJECT
#include <csapex/factory/generic_node.hpp>
#include <csapex/model/node_constructor.h>

/// SYSTEM
#include <boost/function_types/parameter_types.hpp>
#include <boost/type_traits.hpp>

namespace csapex {

class GenericNodeFactory
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
    template<typename F>
    static Node::Ptr wrapFunction(F f)
    {
        typedef typename boost::function_types::parameter_types<F>::type params;

        Node::Ptr result(new GenericNode<params>(f));
        return result;
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
    template<typename F>
    static NodeConstructor::Ptr createConstructorFromFunction(F f,
                                                              const std::string& name,
                                                              const std::string& description,
                                                              Settings& settings,
                                                              const std::vector<TagPtr>& tags = std::vector<TagPtr>(),
                                                              const std::string& icon = ":/no_icon.png"
                                                              )
    {
        return csapex::NodeConstructor::Ptr (new csapex::NodeConstructor(
                                                 settings, name, description, icon, tags,
                                                 boost::bind(&GenericNodeFactory::wrapFunction<F>, f)));
    }
};

}
