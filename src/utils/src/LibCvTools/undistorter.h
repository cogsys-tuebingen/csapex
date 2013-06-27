#ifndef CV_UNDISTORTER_H
#define CV_UNDISTORTER_H
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief The Undistorter class encapsules the functionality for undistortion of images.
 */
class Undistorter
{
public:
    /**
     * @brief Undistorter
     * @param camera_mat            the camera matrix
     * @param distoration_coeffs    the distortation coefficients
     * @param interpolation         the interpolation method to be used
     */
    Undistorter(const cv::Mat &camera_mat, const cv::Mat &distoration_coeffs, const int interpolation = cv::INTER_NEAREST);
    virtual ~Undistorter();
    /**
     * @brief Undistort an image the slow method due to recalculation of undistortion maps.
     * @param src   src image
     * @param dst   dst image
     */
    void undistort_nomap(const cv::Mat &src, cv::Mat &dst);
    /**
     * @brief Undistort an image using undistortion maps.
     * @param src   src image
     * @param dst   dst image
     */
    void undistort(const cv::Mat &src, cv::Mat &dst);
    /**
     * @brief undistort_points_nomap
     * @param src
     * @param dst
     */
    void undistort_points_nomap(const cv::Mat &src, cv::Mat &dst);
    /**
     * @brief undistort_points_nomap
     * @param src
     * @param dst
     */
    void undistort_points_nomap(const std::vector<cv::Point2d> &src, std::vector<cv::Point2d> &dst);
    /**
     * @brief undistort_points_nomap
     * @param src
     * @param dst
     */
    void undistort_points_nomap(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst);
    /**
     * @brief Reset the map and it's size. Method not required for 'nomap' methods
     * @param s - the new size
     */
    void reset_map(const cv::Size s, const float camera_offset_x, const float camera_offset_y);
private:
    int         interpolation_method_;
    cv::Mat     camera_mat_;
    cv::Mat     distortion_coeffs_;
    cv::Mat     optimal_camera_mat_;
    cv::Mat     orientation_;
    cv::Mat     undist_map_x_;
    cv::Mat     undist_map_y_;
    cv::Size    undist_img_size_;
};

#endif // CV_UNDISTORTER_H
