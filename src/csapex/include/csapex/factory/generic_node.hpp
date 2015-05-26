/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message_traits.h>
#include <csapex/model/node_modifier.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/function_types/function_pointer.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/if.hpp>

namespace csapex
{
template <typename Message>
struct GenericInput
{
    typedef Message type;
};
template <typename Message>
struct GenericOutput
{
    typedef Message type;
};
template <typename T>
struct GenericParameter
{
    typedef T type;
};

template <typename Parameters>
class GenericNode : public Node
{
    typedef typename boost::mpl::push_front<Parameters, void>::type Sig;
    typedef typename boost::function_types::function_pointer<Sig>::type Callback;

    enum {
        N = boost::mpl::size< Parameters >::value
    };

public:
    GenericNode(Callback cb)
        : cb_(cb)
    {
        input_.resize(N);
        output_.resize(N);
        params_.resize(N);

        in_msg_.resize(N);
        out_msg_.resize(N);
    }

    virtual void setup(csapex::NodeModifier& modifier)
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeSetup(this, modifier));
    }

    virtual void setupParameters(Parameterizable& params)
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeParameterSetup(this, params));
    }

    void process()
    {
        createMsgs();
        call();
        publish();
    }

private:
    void createMsgs()
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeMessageCreator(this));
    }

    void call();
    void publish()
    {
        for(std::size_t i = 0; i < N; ++i) {
            Output* out = output_[i];
            if(out) {
                msg::publish(out, out_msg_[i]);
            }
        }
    }

public:
    Callback cb_;

    std::vector<Input*> input_;
    std::vector<Output*> output_;
    std::vector<std::string> params_;

    std::vector<std::shared_ptr<ConnectionType const>> in_msg_;
    std::vector<std::shared_ptr<ConnectionType>> out_msg_;

private:
    struct GenericNodeSetup {
        GenericNodeSetup(GenericNode<Parameters>* instance, csapex::NodeModifier& modifier)
            : instance_(instance), modifier_(modifier), id(0)
        {
            instance_->in_msg_.clear();
            instance_->in_msg_.resize(instance_->N);
            instance_->out_msg_.clear();
            instance_->out_msg_.resize(instance_->N);
        }

        template<typename U>
        void operator()(GenericInput<U>) {
            std::string label = connection_types::name<U>();
            instance_->input_[id++] = instance_->modifier_->template addInput<U>(label);
        }
        template<typename U>
        void operator()(GenericOutput<U>) {
            std::string label = connection_types::type<U>::name();
            instance_->output_[id++] = instance_->modifier_->template addOutput<U>(label);

        }
        template<typename U>
        void operator()(GenericParameter<U>) {++id;}

        GenericNode<Parameters>* instance_;
        csapex::NodeModifier& modifier_;
        int id;
    };

    struct GenericNodeParameterSetup {
        GenericNodeParameterSetup(GenericNode<Parameters>* instance, Parameterizable& params)
            : instance_(instance), parameterizable_(params), id(0)
        {}

        template<typename U>
        void operator()(GenericInput<U>) {++id;}
        template<typename U>
        void operator()(GenericOutput<U>) {++id;}

        template<typename U>
        void operator()(GenericParameter<U>) {
            std::string name = "param";
            param::Parameter::Ptr p = param::ParameterFactory::declareValue<U>(name, 0);
            instance_->addParameter(p);
            instance_->params_[id++] = name;
        }

        GenericNode<Parameters>* instance_;
        Parameterizable& parameterizable_;
        int id;
    };

    struct GenericNodeMessageCreator {
        GenericNodeMessageCreator(GenericNode<Parameters>* instance)
            : instance_(instance), id(0)
        {}

        template<typename U>
        void operator()(GenericInput<U>) {
            typename U::ConstPtr in = msg::getMessage<U>(instance_->input_[id]);
            instance_->in_msg_[id] = in;
            ++id;
        }

        template<typename U>
        void operator()(GenericOutput<U>) {
            typename U::Ptr out(connection_types::makeEmptyMessage<U>());
            instance_->out_msg_[id] = out;
            ++id;
        }

        template<typename U>
        void operator()(GenericParameter<U>) {++id;}

        GenericNode<Parameters>* instance_;
        int id;
    };

private:
    struct ClassifyParameter
    {
        template <typename I>
        struct apply
        {
            typedef typename boost::remove_reference<I>::type RawType;

            typedef typename boost::mpl::if_< boost::is_reference<I>,
            typename boost::mpl::if_< std::is_const<RawType>, GenericInput<RawType>, GenericOutput<RawType> >::type,
            GenericParameter<RawType> >::type type;
        };
    };

};


template <typename Parameters>
struct GenerateParameter
{
    template <int index>
    struct Types {
        typedef typename boost::mpl::at<Parameters, boost::mpl::int_<index> >::type type;

        typedef boost::reference_wrapper<typename boost::remove_reference<type>::type> message;

        enum IOType {
            is_const = std::is_const<typename message::type>::value
        };
        typedef type param;
    };

    template <int no>
    static
    typename Types<no>::message
    get(GenericNode<Parameters>* instance,
        typename std::enable_if<
            boost::type_traits::ice_and<
                boost::is_reference<typename Types<no>::type>::value,
                boost::type_traits::ice_not< Types<no>::is_const >::value
            >::value
        >::type* = 0)
    {
        return boost::ref(static_cast<typename Types<no>::type>(*instance->out_msg_[no]));
    }

    template <int no>
    static
    const typename Types<no>::message
    get(GenericNode<Parameters>* instance,
        typename std::enable_if<
            boost::type_traits::ice_and<
                boost::is_reference<typename Types<no>::type>::value,
                Types<no>::is_const
            >::value
        >::type* = 0)
    {
        return boost::ref(static_cast<typename Types<no>::type const>(*instance->in_msg_[no]));
    }

    template <int no>
    static typename Types<no>::param
    get(GenericNode<Parameters>* instance,
        typename std::enable_if<!boost::is_reference<typename Types<no>::type>::value >::type* = 0)
    {
        return instance->template readParameter<int>(instance->params_[no]);
    }
};

// Calling the function pointer
namespace detail {
template <typename ParameterList, int... indices>
struct CallerSentinel
{
    static void call(GenericNode<ParameterList>* i)
    {
        i->cb_(GenerateParameter<ParameterList>::template get<indices>(i)...);
    }
};

template <typename ParameterList, typename Rest, int index, int... indices>
struct Caller
{
    typedef typename Caller<
        ParameterList, typename boost::mpl::pop_front<Rest>::type , index - 1, index, indices...
    >::type type;
};

template <typename ParameterList, typename Rest, int... indices>
struct Caller<ParameterList, Rest, -1, indices...>
{
    typedef CallerSentinel<ParameterList, indices...> type;
};
}

template <typename Parameters>
void GenericNode<Parameters>::call()
{
    detail::Caller<Parameters, Parameters, boost::mpl::size<Parameters>::value - 1>::type::call(this);
}



}
