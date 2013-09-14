/*
 * image_combiner_robust_match.h
 *
 *  Created on: Feb 7, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// COMPONENT
#include <csapex/model/boxed_object.h>

#ifndef IMAGE_COMBINER_ROBUST_MATCH_H
#define IMAGE_COMBINER_ROBUST_MATCH_H

namespace csapex
{

class RobustMatch : public csapex::BoxedObject
{
    Q_OBJECT

public:
    RobustMatch();

    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void allConnectorsArrived();

private:
    ConnectorIn* in_img_1;
    ConnectorIn* in_key_1;
    ConnectorIn* in_des_1;

    ConnectorIn* in_img_2;
    ConnectorIn* in_key_2;
    ConnectorIn* in_des_2;

    ConnectorOut* out_img;
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_ROBUST_MATCH_H
