/// HEADER
#include <csapex/model/node_characteristics.h>

using namespace csapex;

NodeCharacteristics::NodeCharacteristics()
    : depth(-1),
      component(-1),

      is_vertex_separator(false),
      is_joining_vertex(false),
      is_joining_vertex_counterpart(false),
      is_combined_by_joining_vertex(false),
      is_leading_to_joining_vertex(false),

      is_leading_to_essential_vertex(false)
{

}
