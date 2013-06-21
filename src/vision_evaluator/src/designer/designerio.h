#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// SYSTEM
#include <map>
#include <string>

/// FORWARD DECLARATION

namespace vision_evaluator
{
class Box;
class Designer;

class DesignerIO
{
private:
    DesignerIO();

public:
    static void save(Designer* designer);
    static void load(Designer* designer);

private:
    static void loadBoxes(Designer* designer, std::map<std::string, Box*>& loaded_boxes);
    static void loadConnections(Designer* designer, std::map<std::string, Box*>& loaded_boxes);
};

}

#endif // DESIGNERIO_H
