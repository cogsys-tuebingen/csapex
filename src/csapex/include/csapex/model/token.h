#ifndef TOKEN_H
#define TOKEN_H

/// SYSTEM
#include <memory>

namespace csapex {

class Token
{
public:
    typedef std::shared_ptr<Token> Ptr;
    typedef std::shared_ptr<const Token> ConstPtr;

    struct Flags
    {
        enum class Fields {
            MULTI_PART = 1,
            LAST_PART = 2
        };

        Flags();

        u_int8_t data;
    };


public:
    Token(const std::string &type_name);
    virtual ~Token();

    template <typename R>
    std::shared_ptr<R> cloneAs() const
    {
        return std::dynamic_pointer_cast<R>(clone());
    }

    virtual Token::Ptr clone() const = 0;
    virtual Token::Ptr toType() const = 0;

    virtual bool isValid() const;

    virtual bool isContainer() const;
    virtual Ptr nestedType() const;
    virtual ConstPtr nestedValue(std::size_t i) const;
    virtual void addNestedValue(const ConstPtr& msg);
    virtual std::size_t nestedValueCount() const;

    virtual bool canConnectTo(const Token* other_side) const;
    virtual bool acceptsConnectionFrom(const Token *other_side) const;

    virtual std::string descriptiveName() const;
    std::string typeName() const;

    int sequenceNumber() const;
    void setSequenceNumber(int seq_no_) const;

    virtual void writeRaw(const std::string& file,  const std::string &base, const std::string &suffix) const;

protected:
    void setDescriptiveName(const std::string& descriptiveName);

private:
    std::string type_name_;
    std::string descriptive_name_;
    mutable int seq_no_;

public:
    mutable Flags flags;
};

}

#endif // TOKEN_H
