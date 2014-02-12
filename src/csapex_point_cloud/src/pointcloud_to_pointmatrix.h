#ifndef TO_POINT_MATRIX_H
#define TO_POINT_MATRIX_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ToPointMatrix : public csapex::Node
{
public:
    ToPointMatrix();

    virtual void allConnectorsArrived();
    virtual void setup();

};
}
#endif // TO_POINT_MATRIX_H
