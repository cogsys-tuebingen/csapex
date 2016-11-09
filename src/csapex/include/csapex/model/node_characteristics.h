#ifndef NODE_CHARACTERISTICS_H
#define NODE_CHARACTERISTICS_H

namespace csapex
{

class NodeCharacteristics
{
public:
    NodeCharacteristics();

public:
    int depth;
    int component;

    bool is_vertex_separator;

    bool is_joining_vertex;
    bool is_joining_vertex_counterpart;

    bool is_combined_by_joining_vertex;
    bool is_leading_to_joining_vertex;

    bool is_leading_to_essential_vertex;
};

}

#endif // NODE_CHARACTERISTICS_H
