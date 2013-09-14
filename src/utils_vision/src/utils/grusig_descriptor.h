#ifndef GRUSIG_DESCRIPTOR_H
#define GRUSIG_DESCRIPTOR_H

/// SYSTEM
#include <opencv2/features2d/features2d.hpp>

namespace cv
{
/**
 * @brief The GRUSIG class implements the 'Global Ratio of Unique Statistics In Graphics' descriptor.
 */
class GRUSIG : public DescriptorExtractor {
public:
    explicit GRUSIG(int dimension);
    virtual ~GRUSIG();

    /** returns the descriptor length in bytes */
    virtual int descriptorSize() const;

    /** returns the descriptor type */
    virtual int descriptorType() const;

protected:
    virtual void computeImpl( const Mat& image, std::vector<KeyPoint>& keypoints, Mat& descriptors) const;

    void computeRow(const Mat& image, cv::Mat out, KeyPoint& kp) const;
    int cols_;
    int dim_;
};
}

#endif // GRUSIG_DESCRIPTOR_H
