#include "transform_from_models.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>



CSAPEX_REGISTER_CLASS(csapex::TransformFromModels, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFromModels::TransformFromModels()
{
    addTag(Tag::get("Transform"));
}

void TransformFromModels::setup()
{
    setSynchronizedInputs(true);
    input_models_ref_ = addInput<GenericVectorMessage, ModelMessage >("Reference Models");
    input_models_new_ = addInput<GenericVectorMessage, ModelMessage >("New Models");
    output_ = addOutput<connection_types::TransformMessage>("Transformation");
    //output_text_ = addOutput<StringMessage>("String");

    addParameter(param::ParameterFactory::declare("Apex height", 0.0, 2.0, 0.5, 0.01));
    addParameter(param::ParameterFactory::declare("Cone angle", 0.0001, 3.2, 0.5, 0.001));
}

void TransformFromModels::process()
{
    param_apex_height_ = param<double>("Apex height");
    param_cone_angle_ = param<double>("Cone angle");

    // Read Inputs
    boost::shared_ptr<std::vector<ModelMessage> const> models_ref = input_models_ref_->getMessage<GenericVectorMessage, ModelMessage>();
    boost::shared_ptr<std::vector<ModelMessage> const> models_new = input_models_new_->getMessage<GenericVectorMessage, ModelMessage>();


    // Process data
    std::vector<Eigen::Vector3d> points_ref = getInterestingPointsFromModels(models_ref);
    std::vector<Eigen::Vector3d> points_new = getInterestingPointsFromModels(models_new);

    std::cout << "points_ref.size() = " << points_ref.size() << " points_new.size() = " << points_new.size() << std::endl;



//    // Copied from static_transform.cpp remove agian
//    double roll = param<double>("roll");
//    double pitch = param<double>("pitch");
//    double yaw = param<double>("yaw");
//    double x = param<double>("dx");
//    double y = param<double>("dy");
//    double z = param<double>("dz");

    // Publish Output
//    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
//    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
//    output_->publish(msg);
}


std::vector<Eigen::Vector3d> TransformFromModels::getInterestingPointsFromModels(boost::shared_ptr<std::vector<ModelMessage> const> models)
{
    std::vector<Eigen::Vector3d> points;

    // got through all models
    // TODO: there should only be 3 models
    for (std::vector<ModelMessage>::const_iterator it = models->begin(); it != models->end(); it++){
        Eigen::Vector3d interresting_point(0,0,0);
        switch(it->model_type) {
            case pcl::SACMODEL_SPHERE:
            interresting_point(0) = it->coefficients->values.at(0);
            interresting_point(1) = it->coefficients->values.at(1);
            interresting_point(2) = it->coefficients->values.at(2);
                break;

            case pcl::SACMODEL_CONE:
                interresting_point(0) = it->coefficients->values.at(0);
                interresting_point(1) = it->coefficients->values.at(1);
                interresting_point(2) = it->coefficients->values.at(2);
                break;

            case pcl::SACMODEL_CIRCLE2D: {
                interresting_point(0) = it->coefficients->values.at(0);
                interresting_point(1) = it->coefficients->values.at(1);
                // Calculate the z coordiantes of the apex, given the radius and the opening angle
                double radius = it->coefficients->values.at(2);
                double tan_result = std::tan(param_cone_angle_/2.0);
                //assert(tan_result != 0);
                double z;
                if (tan_result != 0) {
                    z = radius / tan_result;
                } else {
                    std::cerr << "Invalid cone angle" << std::endl;
                    z = 0;
                }
                interresting_point(2) = z;
            } break;

            default:
                std::cerr << "Invalid Model Type: " <<  it->model_type << std::endl;
        } // end switch
        points.push_back(interresting_point);
    } // end for

    return points;
}

double TransformFromModels::euklidianDistance(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return std::sqrt((p1(0)-p2(0))*(p1(0)-p2(0)) + (p1(1)-p2(1))*(p1(1)-p2(1)) + (p1(2)-p2(2))*(p1(2)-p2(2)));
}

int TransformFromModels::matchSidesOfTriangles(const std::vector<Eigen::Vector3d> &points1, const std::vector<Eigen::Vector3d> &points2)
{
    // Calculated distances of the points -> that are the sides of the triangles
    std::vector<double> distances1;
    for (unsigned int i=1; i < points1.size(); i++) {
        double distance = euklidianDistance(points1.at(i-1), points1.at(i));
        distances1.push_back(distance);
    }

    std::vector<double> distances2;
    for (unsigned  int i=1; i < points2.size(); i++) {
        double distance = euklidianDistance(points2.at(i-1), points2.at(i));
        distances2.push_back(distance);
    }

    int min_offset = 0;
    if ((distances1.size() > 3) && (distances2.size() > 3)) {
        // find the offset that minimizes the differences of the sides
        double min_sum = 20000.0;
        for (int offset = 0; offset<3; offset++){
            double sum = 0;
            for (int i = 0; i<3; i++) {
                sum += std::pow((distances1.at(i) - distances2.at(i+offset % 3)),2);
            }
            if (sum < min_sum) {
                min_sum = sum;
                min_offset = offset;
            }
        }
    } else  {
        std::cerr << "There is an triangle with less than 3 points (error)" << std::endl;
    }
    return min_offset;
}
