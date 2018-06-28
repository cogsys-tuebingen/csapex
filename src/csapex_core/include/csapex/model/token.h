#ifndef TOKEN_H
#define TOKEN_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/token.h>
#include <csapex/model/clonable.h>
#include <csapex/msg/token_traits.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/model/activity_modifier.h>

namespace csapex {

class CSAPEX_CORE_EXPORT Token : public Clonable
{
protected:
    CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(Token);

public:
    typedef std::shared_ptr<Token> Ptr;

public:
    Token(const TokenDataConstPtr& token);

    void setActivityModifier(ActivityModifier active);
    bool hasActivityModifier() const;
    ActivityModifier getActivityModifier() const;

    TokenDataConstPtr getTokenData() const;

    int getSequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

    virtual void cloneData(const Token& other);

    static Ptr makeEmpty();

private:
    Token();

private:
    TokenDataConstPtr data_;

    ActivityModifier activity_modifier_;

    mutable int seq_no_;
};

}

#endif // TOKEN_CONTAINER_H
