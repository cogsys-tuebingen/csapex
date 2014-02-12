/// HEADER
#include "pointcloud_to_pointmatrix.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>

CSAPEX_REGISTER_CLASS(csapex::ToPointMatrix, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ToPointMatrix::ToPointMatrix()
{
}

void ToPointMatrix::allConnectorsArrived()
{

}

void ToPointMatrix::setup()
{
    setSynchronizedInputs(true);

//    input_ = addInput<PointCloudMessage>("PointCloud");
//    output_ = addOutput<CvMatMessage>("Pointcloud");
}
