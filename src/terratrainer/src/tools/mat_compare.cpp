#include <utils/LibCvTools/terra_mat.h>
#include <iostream>

int main(int argc, char *argv[])
{
    if(argc < 3) {
        std::cerr << "Arguments : <terramat ground truth> <terramat>" << std::endl;
        return 1;
    }


    TerraMat m1;
    TerraMat m2;
    m1.read(argv[1]);
    m2.read(argv[2]);

    cv::Mat val_m1;
    cv::Mat val_m2;
    cv::Mat mat_m1 = m1.getFavorites(val_m1, 0.001f);
    cv::Mat mat_m2 = m1.getFavorites(val_m2, 0.001f);

    int pos = 0;
    int neg = 0;

    for(int i = 0 ; i < mat_m1.rows ; ++i) {
        for(int j = 0 ; j < mat_m1.cols ; ++j) {

            /// ONLY CHECK ON MATCHABLES
            if(val_m1.at<uchar>(i,j) == 1 && val_m2.at<uchar>(i,j) == 1) {
                if(mat_m1.at<uchar>(i,j) == mat_m2.at<uchar>(i,j))
                    pos++;
                else
                    neg++;
            }
        }
    }

    std::cout << "pos : " << pos << " neg :" << neg << std::endl;

    return 0;
}

