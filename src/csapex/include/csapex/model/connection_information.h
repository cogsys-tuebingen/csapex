#ifndef CONNECTION_INFORMATION_H
#define CONNECTION_INFORMATION_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>

namespace csapex
{

struct CSAPEX_EXPORT ConnectionInformation {
    UUID from;
    UUID to;
    std::string from_label;
    std::string to_label;
    TokenDataConstPtr type;

    bool active;

    ConnectionInformation(Connectable* from, Connectable* to, const TokenDataConstPtr &type, bool active);
    ConnectionInformation(const UUID& from, const UUID& to, const TokenDataConstPtr &type, bool active);
};

}

#endif // CONNECTION_INFORMATION_H
