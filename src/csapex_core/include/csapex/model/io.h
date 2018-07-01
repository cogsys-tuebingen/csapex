#ifndef CSAPEX_MODEL_IO_H
#define CSAPEX_MODEL_IO_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/io.h>

namespace csapex
{
namespace model
{
TokenDataConstPtr getTokenData(const TokenPtr& token);

template <typename M>
std::shared_ptr<const M> getTokenData(const TokenPtr& token)
{
    auto data = getTokenData(token);
    return std::dynamic_pointer_cast<const M>(data);
}

}  // namespace model
}  // namespace csapex

#endif  // CSAPEX_MODEL_IO_H
