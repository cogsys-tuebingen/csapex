#include <terra_mat.h>
int main(int argc, char *argv[])
{
    if(argc < 4) {
        std::cerr << "Arguments : <input terra mat> <output image> <cell_size> [<raw mode = 0>]" << std::endl;
        return 1;
    }

    int raw = 0;
    if(argc > 4) {
        raw = atoi(argv[4]);
    }

    int zoom = atoi(argv[3]);


    TerraMat mat;
    mat.read(argv[1]);

    if(mat.getMatrix().empty()) {
        std::cerr << "File couldn't be loaded!" << std::endl;
        return 1;
    }

    cv::Mat render;
    cv::Mat bgr;
    if(raw == 0)
        bgr = mat.getFavoritesBGR();
    else
        bgr = mat.getFavoritesBGRRaw();

    render = cv::Mat(bgr.rows * zoom , bgr.cols * zoom , CV_8UC3, cv::Scalar::all(0));
    for(int i = 0 ; i < bgr.rows ; ++i) {
        for(int j = 0 ; j < bgr.cols ; ++j) {
            cv::Rect rect(j * zoom, i * zoom , zoom, zoom);
            cv::Vec3b  bgr_val = bgr.at<cv::Vec3b>(i,j);
            cv::Scalar color(bgr_val[0] , bgr_val[1], bgr_val[2]);
            cv::rectangle(render, rect, color, CV_FILLED);
        }
    }
    cv::imwrite(argv[2], render);
    return 0;
}
