#ifndef SLIM_SIGNAL_INVOKER_HPP
#define SLIM_SIGNAL_INVOKER_HPP

namespace detail
{
template <int pos, typename Arg, typename... Args>
struct ArgumentExtractor
{
    using type = typename ArgumentExtractor<pos-1, Args...>::type;
};


template <typename Arg, typename... Args>
struct ArgumentExtractor<0, Arg, Args...>
{
    using type = Arg;
};


template <int pos, typename Signature>
struct FunctionArgumentExtractor
{
    using type = Signature;
};

template <int pos, typename Result, typename... Args>
struct FunctionArgumentExtractor<pos, Result(Args...)>
{
    using type = typename ArgumentExtractor<pos, Args...>::type;
};
}

template <int pos, typename Signature>
struct FunctionArgumentExtractor
{
    using type = typename detail::FunctionArgumentExtractor<pos, Signature>::type;
};




template <typename Note, int N, int arg_count>
struct SignalInvoker
{
public:
    template <typename Signature, typename... PartialArgs>
    static void doInvoke(csapex::slim_signal::Signal<Signature>& s, const Note& note, PartialArgs... arguments)
    {
        using ArgN = typename FunctionArgumentExtractor<N, Signature>::type;
        SignalInvoker<Note, N + 1, arg_count - 1>::doInvoke(s, note, arguments..., note.template getPayload<ArgN>(N));
    }
};

template <typename Note, int N>
struct SignalInvoker<Note, N, 0>
{

    template <typename... Args>
    static void doInvoke(csapex::slim_signal::Signal<void(Args...)>& s, const Note& note, Args... arguments)
    {
        s(arguments...);
    }
};


template <typename Note, typename... Args>
static void invokeSignal(csapex::slim_signal::Signal<void(Args...)>& s, const Note& note)
{
    SignalInvoker<Note, 0, sizeof...(Args)>::doInvoke(s, note);
}

#endif // SLIM_SIGNAL_INVOKER_HPP
