#ifndef GAMMA_CPP
#define GAMMA_CPP
#include <opencv2/opencv.hpp>

void correctGamma(const cv::Mat &src, cv::Mat &dst, double gamma ) {
    double inverse_gamma = 1.0 / gamma;

    cv::Mat lut_matrix(1, 256, CV_8UC1 );
    uchar * ptr = lut_matrix.ptr();
    for( int i = 0; i < 256; i++ )
        ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );

    cv::LUT( src, lut_matrix, dst);
}


int main(int argc, char *argv[])
{
    if(argc < 4) {
        std::cerr << "Arguments : <input image> <output image>" << std::endl;
        return 1;
    }

    cv::Mat image = cv::imread(argv[1]);
    if(image.empty()) {
        std::cerr << "Couldn't load image!" << std::endl;
        return 1;
    }

    double gamma = atof(argv[3]);

    correctGamma(image, image, gamma);
    if(cv::imwrite(argv[2], image))
        std::cout << "Congrats you now own a gamma corrected image!" << std::endl;

    return 0;
}

#endif // GAMMA_CPP
