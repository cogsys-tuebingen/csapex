#ifndef BOOLEAN_MESSAGE_H
#define BOOLEAN_MESSAGE_H

/// PROJECT
#include <csapex/message.h>

namespace csapex {
namespace connection_types {


struct BooleanMessage : public MessageTemplate<bool, BooleanMessage>
{
    BooleanMessage();
};

}
}

#endif // BOOLEAN_MESSAGE_H
