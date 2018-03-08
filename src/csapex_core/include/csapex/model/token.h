#ifndef TOKEN_H
#define TOKEN_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token.h>
#include <csapex/msg/token_traits.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/activity_modifier.h>

namespace csapex {

class CSAPEX_CORE_EXPORT Token
{
public:
    Token(const TokenDataConstPtr& token);

    void setActivityModifier(ActivityModifier active);
    bool hasActivityModifier() const;
    ActivityModifier getActivityModifier() const;

    TokenPtr clone() const;

    TokenDataConstPtr getTokenData() const;

    int getSequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

private:
    TokenDataConstPtr token_;

    ActivityModifier activity_modifier_;

    mutable int seq_no_;
};

}

#endif // TOKEN_CONTAINER_H
