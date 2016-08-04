/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/msg/token_traits.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <boost/function_types/function_pointer.hpp>
#include <boost/mpl/for_each.hpp>

namespace csapex
{

namespace generic_node {
struct DefaultInfo
{
    static std::string getName(int /*index*/) {
        return "";
    }

    template <typename P>
    static csapex::param::ParameterPtr declareParameter(int /*index*/) {
        return nullptr;
    }
};
}

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

template <typename Parameters, typename Info>
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

    virtual void setup(csapex::NodeModifier& modifier) override
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeSetup(this, modifier));
    }

    virtual void setupParameters(Parameterizable& params) override
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeParameterSetup(this, params));
    }

    void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& /*parameters*/)
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

    std::vector<std::shared_ptr<TokenData const>> in_msg_;
    std::vector<std::shared_ptr<TokenData>> out_msg_;

private:
    struct GenericNodeSetup {
        GenericNodeSetup(GenericNode<Parameters, Info>* instance, csapex::NodeModifier& modifier)
            : instance_(instance), modifier_(modifier), id(0)
        {
            instance_->in_msg_.clear();
            instance_->in_msg_.resize(instance_->N);
            instance_->out_msg_.clear();
            instance_->out_msg_.resize(instance_->N);
        }

        template<typename U>
        void operator()(GenericInput<U>) {
            std::string label = Info::getName(id);
            if(label.empty()) {
                label = connection_types::serializationName<U>();
            }
            instance_->input_[id++] = instance_->node_modifier_->template addInput<U>(label);
        }
        template<typename U>
        void operator()(GenericOutput<U>) {
            std::string label = Info::getName(id);
            if(label.empty()) {
                label = connection_types::serializationName<U>();
            }
            instance_->output_[id++] = instance_->node_modifier_->template addOutput<U>(label);

        }
        template<typename U>
        void operator()(GenericParameter<U>) {++id;}

        GenericNode<Parameters, Info>* instance_;
        csapex::NodeModifier& modifier_;
        int id;
    };

    struct GenericNodeParameterSetup {
        GenericNodeParameterSetup(GenericNode<Parameters, Info>* instance, Parameterizable& params)
            : instance_(instance), parameterizable_(params), id(0)
        {}

        template<typename U>
        void operator()(GenericInput<U>) {++id;}
        template<typename U>
        void operator()(GenericOutput<U>) {++id;}

        template<typename U>
        void operator()(GenericParameter<U>) {
            std::string name = Info::getName(id);
            if(name.empty()) {
                name =  std::string("param ") + std::to_string(id);
            }
            csapex::param::Parameter::Ptr p = Info::template declareParameter<U>(id);
            if(p == nullptr) {
                p = csapex::param::ParameterFactory::declareValue<U>(name, 0);
            }
            instance_->addParameter(p);
            instance_->params_[id] = name;

            ++id;
        }

        GenericNode<Parameters, Info>* instance_;
        Parameterizable& parameterizable_;
        int id;
    };

    struct GenericNodeMessageCreator {
        GenericNodeMessageCreator(GenericNode<Parameters, Info>* instance)
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

        GenericNode<Parameters, Info>* instance_;
        int id;
    };

private:
    struct ClassifyParameter
    {
        template <typename I>
        struct apply
        {
            typedef typename std::decay<I>::type RawType;
            typedef typename connection_types::MessageContainer<RawType>::type Msg;

            static_assert(!std::is_pointer<I>::value, "type is not a pointer");
            static_assert(std::is_base_of<TokenData, Msg>::value ||
                          std::is_integral<I>::value ||
                          std::is_floating_point<I>::value ||
                          std::is_same<std::string, Msg>::value,
                          "type is usable");

            typedef typename boost::mpl::if_< std::is_reference<I>,
            typename boost::mpl::if_<
                std::is_const<typename std::remove_reference<I>::type>,
                    GenericInput<Msg>,
                    GenericOutput<Msg> >::type,
                GenericParameter<RawType>
            >::type type;
        };
    };

};


template <typename Parameters, typename Info>
struct GenerateParameter
{
    template <int index>
    struct Types {
        typedef typename boost::mpl::at<Parameters, boost::mpl::int_<index> >::type type;
        typedef typename std::decay<type>::type decay_type;

        typedef std::reference_wrapper<typename boost::remove_reference<type>::type> message;

        enum IOType {
            is_const = std::is_const<typename message::type>::value
        };
        typedef type param;
    };

    template <int no>
    static
    typename Types<no>::message
    get(GenericNode<Parameters, Info>* instance,
        typename std::enable_if<
        std::is_reference<typename Types<no>::type>::value &&
        !Types<no>::is_const
        >::type* = 0)
    {
        typedef typename Types<no>::decay_type dt;
        typedef typename connection_types::MessageContainer<dt>::type msg_t;
        typedef typename Types<no>::type expected;

        msg_t& msg = dynamic_cast<msg_t&>(*instance->out_msg_[no]);
        auto& val = connection_types::MessageContainer<dt>::access(msg);
        return std::ref(static_cast<expected>(val));
    }

    template <int no>
    static
    const typename Types<no>::message
    get(GenericNode<Parameters, Info>* instance,
        typename std::enable_if
        <
        std::is_reference<typename Types<no>::type>::value &&
        Types<no>::is_const
        >::type* = 0)
    {
        typedef typename Types<no>::decay_type dt;
        typedef typename connection_types::MessageContainer<dt>::type msg_t;
        typedef typename Types<no>::type expected;

        const msg_t& msg = dynamic_cast<const msg_t&>(*instance->in_msg_[no]);
        auto& val = connection_types::MessageContainer<dt>::accessConst(msg);
        return std::ref(static_cast<expected const>(val));
    }

    template <int no>
    static typename Types<no>::param
    get(GenericNode<Parameters, Info>* instance,
        typename std::enable_if<!std::is_reference<typename Types<no>::type>::value >::type* = 0)
    {
        return instance->template readParameter<typename Types<no>::decay_type>(instance->params_[no]);
    }
};

// Calling the function pointer
namespace detail {
template <typename ParameterList, typename Info, int... indices>
struct CallerSentinel
{
    static void call(GenericNode<ParameterList, Info>* i)
    {
        i->cb_(GenerateParameter<ParameterList, Info>::template get<indices>(i)...);
    }
};

template <typename ParameterList, typename Info, typename Rest, int index, int... indices>
struct Caller
{
    typedef typename Caller<
        ParameterList, Info, typename boost::mpl::pop_front<Rest>::type , index - 1, index, indices...
    >::type type;
};

template <typename ParameterList, typename Info, typename Rest, int... indices>
struct Caller<ParameterList, Info, Rest, -1, indices...>
{
    typedef CallerSentinel<ParameterList, Info, indices...> type;
};
}

template <typename Parameters, typename Info>
void GenericNode<Parameters, Info>::call()
{
    detail::Caller<Parameters, Info, Parameters, boost::mpl::size<Parameters>::value - 1>::type::call(this);
}



}
