#ifndef TOKEN_DATA_H
#define TOKEN_DATA_H

/// COMPONENT
#include <csapex_core/csapex_core_export.h>
#include <csapex/serialization/serializable.h>

/// SYSTEM
#include <memory>
#include <string>

namespace csapex {

class CSAPEX_CORE_EXPORT TokenData : public Serializable
{
public:
    typedef std::shared_ptr<TokenData> Ptr;
    typedef std::shared_ptr<const TokenData> ConstPtr;

public:
    TokenData(const std::string &type_name);
    TokenData(const std::string &type_name, const std::string& descriptive_name);
    virtual ~TokenData();

    static TokenData::Ptr makeEmpty() {
        return std::shared_ptr<TokenData>(new TokenData);
    }

    virtual TokenData::Ptr toType() const;

    template <typename R>
    std::shared_ptr<R> cloneAs() const
    {
        return std::dynamic_pointer_cast<R>(clone());
    }


    virtual bool isValid() const;

    virtual bool isContainer() const;
    virtual Ptr nestedType() const;
    virtual ConstPtr nestedValue(std::size_t i) const;
    virtual void addNestedValue(const ConstPtr& msg);
    virtual std::size_t nestedValueCount() const;

    virtual bool canConnectTo(const TokenData* other_side) const;
    virtual bool acceptsConnectionFrom(const TokenData *other_side) const;

    virtual std::string descriptiveName() const;
    std::string typeName() const;

    virtual void writeRaw(const std::string& file,  const std::string &base, const std::string &suffix) const;

    virtual TokenData::Ptr clone() const;
    std::shared_ptr<Clonable> makeEmptyClone() const override;
    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;


protected:
    TokenData();
    void setDescriptiveName(const std::string& descriptiveName);

private:
    std::string type_name_;
    std::string descriptive_name_;
};

}

#endif // TOKEN_H
