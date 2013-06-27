#ifndef CV_PERSPECTIVE_TRANSFORM_H
#define CV_PERSPECTIVE_TRANSFORM_H
#include <opencv2/core/core.hpp>

/**
 * @brief The PerspectiveTransform class calculates a perspective transformation
 *        given different parameters, such as 3 angles, focal length and distance.
 */
class PerspectiveTransform
{
public:
    /**
     * @brief PerspectiveTransform constructor.
     */
    PerspectiveTransform();
    /**
     * @brief ~PerspectiveTransform destructor.
     */
    virtual ~PerspectiveTransform();
    /**
     * @brief Set rotation around x axis using angles.
     * @param ang   the angle
     */
    void set_rot_x(const double ang);
    /**
     * @brief Set rotation around y axis using angles.
     * @param ang   the angle
     */
    void set_rot_y(const double ang);
    /**
     * @brief Set rotation around z axis using angles.
     * @param ang   the angle
     */
    void set_rot_z(const double ang);
    /**
     * @brief Set rotation around x axis using radian.
     * @param rad   the radian
     */
    void set_rot_x_rad(const double rad);
    /**
     * @brief Set rotation around y axis using radian.
     * @param rad   the radian
     */
    void set_rot_y_rad(const double rad);
    /**
     * @brief Set rotation around z axis using radian.
     * @param rad   the radian
     */
    void set_rot_z_rad(const double rad);
    /**
     * @brief Set the distance to an image.
     * @param dist  the distance
     */
    void set_distance(const double dist);
    /**
     * @brief Set teh virtual focal length.
     * @param focal the focal length
     */
    void set_focal(const double focal);
    /**
     * @brief Transform an image.
     * @param src   source image
     * @param dst   destination image
     */
    void transform(const cv::Mat &src, cv::Mat &dst);

private:
    bool              dirty_;

    cv::Mat           proj_to3d_;
    cv::Mat           proj_to2d_;
    cv::Mat           distance_;
    cv::Mat           rot_x_;
    cv::Mat           rot_y_;
    cv::Mat           rot_z_;
    cv::Size          img_size_;

    cv::Mat           transformation_;

    void init();
    void recompute_transform();

};

#endif // CV_PERSPECTIVE_TRANSFORM_H
