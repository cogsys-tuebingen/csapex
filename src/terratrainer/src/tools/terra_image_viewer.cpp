#include <utils/LibCvTools/terra_mat.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

using namespace boost;

int main(int argc, char *argv[])
{
    std::string path = argv[1];

    if(argc == 1) {
        std::cout << "Arguments : <terrr_mat> [<zoom>]" << std::endl;
        return 1;
    }


    if(!boost::filesystem3::exists(path)) {
        std::cerr << "File does not exist '" << path << "' !" << std::endl;
        return 1;
    }

    boost::regex e_yaml(".*\\.(yml)");
    std::vector<std::string> paths;
    if(filesystem3::is_directory(path)) {
        filesystem3::directory_iterator end_iter;
        for (filesystem3::directory_iterator dir_itr( path ); dir_itr != end_iter; ++dir_itr) {
            try {
                boost::cmatch what;
                if (filesystem3::is_regular_file( dir_itr->status())) {
                    std::string current = dir_itr->path().string();
                    if(boost::regex_match(current.c_str(), what, e_yaml) && what[0].matched) {
                        paths.push_back(current);
                    }
                }
            }
            catch ( const std::exception & ex ){
                std::cerr << dir_itr->path().filename() << " " << ex.what() << std::endl;
            }
        }
    } else {
        paths.push_back(path);
    }

    int zoom = 1;
    if(argc > 2) {
        zoom = atoi(argv[2]);
    }

    /// up : 1113938 65362
    /// down : 1113940 65362
    /// esc  : 27 1048603

    if(paths.size() == 0) {
        std::cerr << "No paths found !" << std::endl;
        return 1;
    }

    cv::namedWindow("Generation");
    bool end = false;
    int  pos = 0;
    while(!end) {
        TerraMat mat;
        cv::Mat render;
        mat.read(paths[pos]);

        if(!mat.getMatrix().empty()) {
            cv::Mat rgb = mat.getFavoritesBGR();
            render = cv::Mat(rgb.rows * zoom , rgb.cols * zoom , CV_8UC3, cv::Scalar::all(0));
            for(int i = 0 ; i < rgb.rows ; i++) {
                for(int j = 0 ; j < rgb.cols ; j++) {
                    cv::Rect rect(j * zoom, i * zoom , zoom, zoom);
                    cv::Vec3b  bgr_val = rgb.at<cv::Vec3b>(i,j);
                    cv::Scalar color(bgr_val[0] , bgr_val[1], bgr_val[2]);
                    cv::rectangle(render, rect, color, CV_FILLED);
                }

            }
        } else {
            render = cv::Mat(480,640, CV_8UC3);
        }

        cv::imshow("Generation", render);
        int key = cv::waitKey(0);

        end = key == 27 || key == 1048603;
        switch(key) {
        case 27:
            end = true;
            break;
        case 1048603:
            end = true;
            break;
        case 1113938:
            pos--;
            break;
        case 65362:
            pos--;
        break;
        case 1113940:
            pos++;
            break;
        case 65364:
            pos++;
            break;
        }

        pos %= paths.size();

    }

    cv::destroyWindow("Generation");
    return 0;
}

