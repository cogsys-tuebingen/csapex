#ifndef CLIPBOARD_H
#define CLIPBOARD_H

/// SYSTEM
#include <string>

namespace YAML
{
class Node;
}

namespace csapex
{

class ClipBoard
{

public:
    static bool canPaste();

    static void set(const YAML::Node& serialized);
    static std::string get();
};

}

#endif // CLIPBOARD_H
