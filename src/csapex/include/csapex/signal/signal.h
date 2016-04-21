#ifndef SIGNAL_H
#define SIGNAL_H

/// COMPONENT
#include <csapex/msg/message.h>

namespace csapex {
namespace connection_types {

class Signal : public Token
{
public:
    typedef std::shared_ptr<Signal> Ptr;

    Signal();

    virtual Token::Ptr clone() const override;
    virtual Token::Ptr toType() const override;

    bool acceptsConnectionFrom(const Token* other_side) const override;
};


/// TRAITS
template <>
struct type<Signal> {
    static std::string name() {
        return std::string("Signal");
    }
};

}
}

/// YAML
namespace YAML {
template <>
struct convert<csapex::connection_types::Signal> {
  static Node encode(const csapex::connection_types::Signal&);
  static bool decode(const Node&, csapex::connection_types::Signal&);
};
}

#endif // SIGNAL_H
