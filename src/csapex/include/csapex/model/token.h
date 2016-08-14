#ifndef TOKEN_H
#define TOKEN_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token.h>
#include <csapex/msg/token_traits.h>
#include "csapex_export.h"

namespace csapex {

class CSAPEX_EXPORT Token
{
public:
    struct Flags
    {
        enum class Fields {
            MULTI_PART = 8,
            LAST_PART = 16,

        };

        Flags();

        std::int8_t data;
    };



public:
    template <typename DataType>
    static TokenPtr makeEmpty() {
        return std::make_shared<Token>(connection_types::makeEmpty<DataType>());
    }

public:
    Token(const TokenDataConstPtr& token);

    void setActive(bool active);
    bool isActive() const;

    TokenPtr clone() const;

    TokenDataConstPtr getTokenData() const;

    int getSequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

private:
    TokenDataConstPtr token_;

    bool active_;

    mutable int seq_no_;

public:
    mutable Flags flags;
};

}

#endif // TOKEN_CONTAINER_H
