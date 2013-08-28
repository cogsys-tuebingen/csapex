#ifndef LBP_HPP
#define LBP_HPP
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

using namespace cv;

namespace cv_local_patterns {

class LocalPattern {
public:
    typedef boost::shared_ptr<LocalPattern> Ptr;
protected :
    LocalPattern(){}
};

class LBP : public LocalPattern
{
public:
    typedef boost::shared_ptr<LBP> Ptr;

    LBP() :
        histogram(cv::Mat_<int>(1, 256, 0))
    {
    }

    double operator - (const LBP &other) const
    {
        double sum = 0;
        for(int i = 0 ; i < histogram.rows ; i) {
            int diff = histogram.at<int>(i) - other.histogram.at<int>(i);
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    cv::Mat getHistogram()
    {
        return histogram.clone();
    }

    template <typename _Tp>
    void stdExtraction(InputArray _src, const _Tp k = 0) {
        histogram.setTo(0);
        // get matrices
        Mat src = _src.getMat();
        // calculate patterns
        for(int i=1;i<src.rows-1;i++) {
            for(int j=1;j<src.cols-1;j++) {
                _Tp center = src.at<_Tp>(i,j) + k;
                unsigned char histgram_pos = 0;

                histgram_pos += (src.at<_Tp>(i-1,j-1)   >= center) << 7;
                histgram_pos += (src.at<_Tp>(i-1,j)     >= center) << 6;
                histgram_pos += (src.at<_Tp>(i-1,j+1)   >= center) << 5;
                histgram_pos += (src.at<_Tp>(i,j+1)     >= center) << 4;
                histgram_pos += (src.at<_Tp>(i+1,j+1)   >= center) << 3;
                histgram_pos += (src.at<_Tp>(i+1,j)     >= center) << 2;
                histgram_pos += (src.at<_Tp>(i+1,j-1)   >= center) << 1;
                histgram_pos += (src.at<_Tp>(i,j-1)     >= center) << 0;

                histogram.at<int>(histgram_pos)++;
            }
        }
    }

    template <typename _Tp>
    inline void extExtraction(InputArray _src, const int radius, const int neighbors, const _Tp k = 0) {
        //get matrices
        Mat src = _src.getMat();
        for(int n=0; n<neighbors; n++) {
            // sample points
            float x = static_cast<float>(-radius * sin(2.0*CV_PI*n/static_cast<float>(neighbors)));
            float y = static_cast<float>(radius * cos(2.0*CV_PI*n/static_cast<float>(neighbors)));
            // relative indices
            int fx = static_cast<int>(floor(x));
            int fy = static_cast<int>(floor(y));
            int cx = static_cast<int>(ceil(x));
            int cy = static_cast<int>(ceil(y));
            // fractional part
            float ty = y - fy;
            float tx = x - fx;
            // set interpolation weights
            float w1 = (1 - tx) * (1 - ty);
            float w2 =      tx  * (1 - ty);
            float w3 = (1 - tx) *      ty;
            float w4 =      tx  *      ty;
            // iterate through your data
            for(int i=radius; i < src.rows-radius;i++) {
                for(int j=radius;j < src.cols-radius;j++) {
                    // calculate interpolated value
                    float t = static_cast<float>(w1*src.at<_Tp>(i+fy,j+fx) + w2*src.at<_Tp>(i+fy,j+cx) + w3*src.at<_Tp>(i+cy,j+fx) + w4*src.at<_Tp>(i+cy,j+cx));
                    // floating point precision, so check some machine-dependent epsilon
                    _Tp center = src.at<_Tp>(i,j) + k;
                    unsigned char histogram_pos = 0;
                    histogram_pos += (t >= center) << n;
                    histogram.at<int>(histogram_pos)++;
                }
            }
        }
    }


private:
    cv::Mat histogram;
};

class LTP  : public LocalPattern
{
public :
    typedef boost::shared_ptr<LTP> Ptr;

    LTP() :
        pos_(cv::Mat_<int>(1, 256, 0)),
        neg_(cv::Mat_<int>(1, 256, 0))
    {
    }

    double operator - (const LTP &other) const
    {
        double sum = 0;
        for(int i = 0 ; i < pos_.rows ; i) {
            int pos_diff = pos_.at<int>(i) - other.pos_.at<int>(i);
            int neg_diff = neg_.at<int>(i) - other.neg_.at<int>(i);
            sum += pos_diff * pos_diff + neg_diff * neg_diff;
        }
        return std::sqrt(sum);
    }

    cv::Mat getPos()
    {
        return pos_.clone();
    }

    cv::Mat getNeg()
    {
        return neg_.clone();
    }

    template <typename _Tp>
    void stdExtraction(InputArray _src, const _Tp k) {
        pos_.setTo(0);
        neg_.setTo(0);
        // get matrices
        Mat src = _src.getMat();
        // calculate patterns
        for(int i=1;i<src.rows-1;i++) {
            for(int j=1;j<src.cols-1;j++) {
                _Tp center = src.at<_Tp>(i,j);
                unsigned char hist_neg_it = 0;
                unsigned char hist_pos_it = 0;
                _Tp center_minu_k = center - k;
                _Tp center_plus_k = center + k;

                hist_pos_it += (src.at<_Tp>(i-1,j-1)   >= center_plus_k) << 7;
                hist_neg_it += (src.at<_Tp>(i-1,j-1)    < center_minu_k) << 7;
                hist_pos_it += (src.at<_Tp>(i-1,j)     >= center_plus_k) << 6;
                hist_neg_it += (src.at<_Tp>(i-1,j)      < center_minu_k) << 6;
                hist_pos_it += (src.at<_Tp>(i-1,j+1)   >= center_plus_k) << 5;
                hist_neg_it += (src.at<_Tp>(i-1,j+1)    < center_minu_k) << 5;
                hist_pos_it += (src.at<_Tp>(i,j+1)     >= center_plus_k) << 4;
                hist_neg_it += (src.at<_Tp>(i,j+1)      < center_minu_k) << 4;
                hist_pos_it += (src.at<_Tp>(i+1,j+1)   >= center_plus_k) << 3;
                hist_neg_it += (src.at<_Tp>(i+1,j+1)    < center_minu_k) << 3;
                hist_pos_it += (src.at<_Tp>(i+1,j)     >= center_plus_k) << 2;
                hist_neg_it += (src.at<_Tp>(i+1,j)      < center_minu_k) << 2;
                hist_pos_it += (src.at<_Tp>(i+1,j-1)   >= center_plus_k) << 1;
                hist_neg_it += (src.at<_Tp>(i+1,j-1)    < center_minu_k) << 1;
                hist_pos_it += (src.at<_Tp>(i,j-1)     >= center_plus_k) << 0;
                hist_neg_it += (src.at<_Tp>(i,j-1)      < center_minu_k) << 0;

                neg_.at<int>(hist_neg_it)++;
                pos_.at<int>(hist_pos_it)++;
            }
        }
    }

    template <typename _Tp>
    inline void extExtraction(InputArray _src, const int radius, const int neighbors, const _Tp k) {
        //get matrices
        Mat src = _src.getMat();
        for(int n=0; n<neighbors; n++) {
            // sample points
            float x = static_cast<float>(-radius * sin(2.0*CV_PI*n/static_cast<float>(neighbors)));
            float y = static_cast<float>(radius * cos(2.0*CV_PI*n/static_cast<float>(neighbors)));
            // relative indices
            int fx = static_cast<int>(floor(x));
            int fy = static_cast<int>(floor(y));
            int cx = static_cast<int>(ceil(x));
            int cy = static_cast<int>(ceil(y));
            // fractional part
            float ty = y - fy;
            float tx = x - fx;
            // set interpolation weights
            float w1 = (1 - tx) * (1 - ty);
            float w2 =      tx  * (1 - ty);
            float w3 = (1 - tx) *      ty;
            float w4 =      tx  *      ty;
            // iterate through your data
            for(int i=radius; i < src.rows-radius;i++) {
                for(int j=radius;j < src.cols-radius;j++) {
                    // calculate interpolated value
                    float t = static_cast<float>(w1*src.at<_Tp>(i+fy,j+fx) + w2*src.at<_Tp>(i+fy,j+cx) + w3*src.at<_Tp>(i+cy,j+fx) + w4*src.at<_Tp>(i+cy,j+cx));
                    // floating point precision, so check some machine-dependent epsilon
                    _Tp center = src.at<_Tp>(i,j);
                    unsigned char hist_neg_it = 0;
                    unsigned char hist_pos_it = 0;
                    _Tp center_minu_k = center - k;
                    _Tp center_plus_k = center + k;
                    hist_pos_it += (t   >= center_plus_k) << n;
                    hist_neg_it += (t    < center_minu_k) << n;
                    neg_.at<int>(hist_neg_it)++;
                    pos_.at<int>(hist_pos_it)++;
                }
            }
        }
    }

private:
    cv::Mat pos_;
    cv::Mat neg_;

};
}
#endif // LBP_HPP
