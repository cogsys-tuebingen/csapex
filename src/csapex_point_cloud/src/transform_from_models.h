#ifndef TRANSFORM_FROM_MODELS_H
#define TRANSFORM_FROM_MODELS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/model_message.h>


namespace csapex {

class TransformFromModels : public csapex::Node
{
public:
    TransformFromModels();

    virtual void process();
    virtual void setup();
private:
    ConnectorIn* input_models_ref_;
    ConnectorIn* input_models_new_;
    ConnectorOut* output_;

    double param_apex_height_;
    double param_cone_angle_;

    /** Get interresting points from models
     * @brief This function takes a vector of models and returns a vector of coordinate of the needed points for further calcualtion
     *        For a cone it returns the apex corrdinates for the circle it calculates also the apex coordinates of the cone
     * @param[in] models a vector with all the models of the segmentation
     * @return a vector with the cordinates of the interresting points
      */
    std::vector<Eigen::Vector3d> getInterestingPointsFromModels(boost::shared_ptr<std::vector<ModelMessage> const> models);

    /** Match sides of triangle
     * @brief compares the sides of a triangle defined by 3 points
     * @param points1 three points of the first triangle
     * @param points2 three points of the second triangle
     *
     * @return the shift offset that makes the points match
     */
    int matchSidesOfTriangles(const std::vector<Eigen::Vector3d> &points1, const std::vector<Eigen::Vector3d> &points2);

    /** Calculate the euklidian distance of two points in 3d Space
     * @return euklidian distance in meter
     */
    double euklidianDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);

    /** calculate euler angles from a Rotationmatrix R
     * @brief calculates the euler angles from a rotatin matrix R based on the example from
     * Slabaugh http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf
     * @param[in] R  3 x 3 rotation matrix
     * @param[out] psi out roation angle around x-axis
     * @param[out] theta out roation angle around y-axis
     * @param[out] phi roation angle around z-axis
     */
    void eulerAnglesFromRotationMatrix( Eigen::Matrix3d R, double &psi, double &theta, double &phi);

    // Find Transformation from matched triangels
    Eigen::Matrix4d calculateTransformation(const std::vector<Eigen::Vector3d> &points_ref, const std::vector<Eigen::Vector3d> &points_new, int offset);
    Eigen::Matrix4d threePointsToTransformation(const std::vector<Eigen::Vector3d> &points);
    // Optimize transform over time see: http://www.cs.cmu.edu/~ranjith/lcct.html
};
}
#endif // TRANSFORM_FROM_MODELS_H

