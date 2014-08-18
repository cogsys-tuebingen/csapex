/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/message_traits.h>
#include <csapex/model/node_modifier.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/function_types/function_pointer.hpp>
#include <boost/mpl/for_each.hpp>

namespace csapex
{
template <typename Message>
struct GenericInput
{
    typedef Message type;
    typedef typename boost::remove_const<Message>::type RawType;
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

    template <int argc>
    friend class Caller;

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

        msg_.resize(N);
    }

    void setup()
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeSetup(this));
    }

    void setupParameters()
    {
        boost::mpl::for_each<Parameters, ClassifyParameter>(GenericNodeParameterSetup(this));
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
                out->publish(msg_[i]);
            }
        }
    }

public:
    Callback cb_;

    std::vector<Input*> input_;
    std::vector<Output*> output_;
    std::vector<std::string> params_;

    std::vector<ConnectionTypePtr> msg_;

private:
    struct GenericNodeSetup {
        GenericNodeSetup(GenericNode<Parameters>* instance)
            : instance_(instance), id(0)
        {
            instance_->msg_.clear();
            instance_->msg_.resize(instance_->N);
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
        int id;
    };

    struct GenericNodeParameterSetup {
        GenericNodeParameterSetup(GenericNode<Parameters>* instance)
            : instance_(instance), id(0)
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
        int id;
    };

    struct GenericNodeMessageCreator {
        GenericNodeMessageCreator(GenericNode<Parameters>* instance)
            : instance_(instance), id(0)
        {}

        template<typename U>
        void operator()(GenericInput<U>) {
            typename U::Ptr in = instance_->input_[id]->template getMessage<typename GenericInput<U>::RawType>();
            instance_->msg_[id] = in;
            ++id;
        }

        template<typename U>
        void operator()(GenericOutput<U>) {
            typename U::Ptr out(connection_types::makeEmpty<U>());
            instance_->msg_[id] = out;
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
            typename boost::mpl::if_< boost::is_const<RawType>, GenericInput<RawType>, GenericOutput<RawType> >::type,
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
        typedef type param;
    };

    template <int no>
    static
    typename Types<no>::message
    get(GenericNode<Parameters>* instance,
        typename boost::enable_if<boost::is_reference<typename Types<no>::type> >::type* = 0)
    {
        return boost::ref(static_cast<typename Types<no>::type>(*instance->msg_[no]));
    }
    template <int no>
    static typename Types<no>::param
    get(GenericNode<Parameters>* instance,
        typename boost::disable_if<boost::is_reference<typename Types<no>::type> >::type* = 0)
    {
        return instance->template readParameter<int>(instance->params_[no]);
    }
};

template <int argc>
struct Caller
{
};

#define MAKE_CALLER(params, PARAMS) \
    template <> \
    struct Caller<params>\
{\
    template <typename P>\
    static void call(GenericNode<P>* i) {\
    boost::bind(PARAMS)();\
}\
};

#define GET(no)\
    GenerateParameter<P>::template get<no>(i)

#define CALL_0 i->cb_
#define CALL_1 CALL_0, GET(0)
#define CALL_2 CALL_1, GET(1)
#define CALL_3 CALL_2, GET(2)
#define CALL_4 CALL_3, GET(3)
#define CALL_5 CALL_4, GET(4)
#define CALL_6 CALL_5, GET(5)
#define CALL_7 CALL_6, GET(6)
#define CALL_8 CALL_7, GET(7)
#define CALL_9 CALL_8, GET(8)

#define CALLER(params) \
    MAKE_CALLER(params, CALL_##params)

CALLER(1)
CALLER(2)
CALLER(3)
CALLER(4)
CALLER(5)
CALLER(6)
CALLER(7)
CALLER(8)
CALLER(9)

template <typename Parameters>
void GenericNode<Parameters>::call()
{
    Caller < boost::mpl::size< Parameters >::value >::call(this);
}



}
