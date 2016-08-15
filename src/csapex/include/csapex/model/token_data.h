#ifndef TOKEN_DATA_H
#define TOKEN_DATA_H

/// COMPONENT
#include <csapex/csapex_export.h>

/// SYSTEM
#include <memory>
#include <string>

namespace csapex {

class CSAPEX_EXPORT TokenData
{
public:
    typedef std::shared_ptr<TokenData> Ptr;
    typedef std::shared_ptr<const TokenData> ConstPtr;

public:
    TokenData(const std::string &type_name);
    virtual ~TokenData();

    virtual TokenData::Ptr clone() const;
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


protected:
    void setDescriptiveName(const std::string& descriptiveName);

private:
    std::string type_name_;
    std::string descriptive_name_;
};

}

#endif // TOKEN_H
