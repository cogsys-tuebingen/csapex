#ifndef CLIPBOARD_H
#define CLIPBOARD_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <string>

namespace YAML
{
class Node;
}

namespace csapex
{

class CSAPEX_QT_EXPORT ClipBoard
{

public:
    static bool canPaste();

    static void set(const YAML::Node& serialized);
    static std::string get();
};

}

#endif // CLIPBOARD_H
