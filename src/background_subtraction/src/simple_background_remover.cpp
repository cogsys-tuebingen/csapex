/// HEADER
#include "simple_background_remover.h"

using namespace background_subtraction;

SimpleBackgroundRemover::SimpleBackgroundRemover()
{
}

void SimpleBackgroundRemover::segmentation(const cv::Mat& frame, cv::Mat& mask)
{
    cv::Mat blured;
    cv::GaussianBlur(frame, blured, blur_kernel, sigma_blur);

    uchar* mask_data = mask.data;
    uchar* frame_data = blured.data;
    uchar* bg_data = background_blured.data;

    unsigned channels = blured.channels();
    int ws  = channels * blured.cols * sizeof(uchar);
    int ws_out = mask.cols * sizeof(uchar);

    double max_dist = difference_threshold;
    double max_dist_sqr = max_dist*max_dist;

    // segmentation
    for (int i = 0; i < blured.rows; i++) {
        for (int j = 0; j < blured.cols; j++) {
            int jc = j * channels;
            unsigned char f_r = frame_data[i * ws + jc + 2];
            unsigned char f_g = frame_data[i * ws + jc + 1];
            unsigned char f_b = frame_data[i * ws + jc];

            unsigned char bg_r = bg_data[i * ws + jc + 2];
            unsigned char bg_g = bg_data[i * ws + jc + 1];
            unsigned char bg_b = bg_data[i * ws + jc];

            int dr = f_r - bg_r;
            int dg = f_g - bg_g;
            int db = f_b - bg_b;

            double dist_sqr = (dr*dr + dg*dg + db*db);

            if(dist_sqr < max_dist_sqr) {
                mask_data[i * ws_out + j] = 0;
            } else {
                mask_data[i * ws_out + j] = 255;
            }
        }
    }
}
