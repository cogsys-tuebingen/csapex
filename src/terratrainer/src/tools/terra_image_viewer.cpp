#include <utils/LibCvTools/terra_mat.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

using namespace boost;

int brush_width = 0;

std::vector<cv::Vec<int,5> > changes;

void mouse_event(int event, int x , int y , int flags, void *param)
{
    cv::Vec<int,5> *vec = (cv::Vec<int,5>*) param;

    bool drag = event == CV_EVENT_MOUSEMOVE && (*vec)[4] == 1;

    if(event == CV_EVENT_LBUTTONDOWN || event == CV_EVENT_RBUTTONDOWN || drag) {
        (*vec)[0] = x;
        (*vec)[1] = y;
        (*vec)[3] = 1;
        (*vec)[4] = 1;

        if(event == CV_EVENT_LBUTTONDOWN)
            brush_width = 0;
        if(event == CV_EVENT_RBUTTONDOWN)
            brush_width = 1;
    }

    if(event == CV_EVENT_LBUTTONUP || event == CV_EVENT_RBUTTONUP)
        (*vec)[4] = 0;

    if(event == CV_EVENT_LBUTTONDOWN || event == CV_EVENT_RBUTTONDOWN || drag) {
        changes.push_back(cv::Vec<int,5>(*vec));
    }
}

int main(int argc, char *argv[])
{
    std::string path = argv[1];

    if(argc == 1) {
        std::cout << "Arguments : <terrr_mat> [<zoom>] [<raw>]" << std::endl;
        return 1;
    }

    std::cerr << "WARNING: Saving corrected matrices will override the original!" << std::endl;


    if(!boost::filesystem3::exists(path)) {
        std::cerr << "File does not exist '" << path << "' !" << std::endl;
        return 1;
    }

    boost::regex e_yaml(".*\\.(yml)");
    boost::regex e_jpg(".*\\.(jpg)");
    std::vector<std::string> yml_paths;
    std::vector<std::string> jpg_paths;
    if(filesystem3::is_directory(path)) {
        filesystem3::directory_iterator end_iter;
        for (filesystem3::directory_iterator dir_itr( path ); dir_itr != end_iter; ++dir_itr) {
            try {
                boost::cmatch what;
                if (filesystem3::is_regular_file( dir_itr->status())) {
                    std::string current = dir_itr->path().string();
                    if(boost::regex_match(current.c_str(), what, e_yaml) && what[0].matched) {
                        yml_paths.push_back(current);
                    }
                    if(boost::regex_match(current.c_str(), what, e_jpg) && what[0].matched) {
                        jpg_paths.push_back(current);
                    }
                }
            }
            catch ( const std::exception & ex ){
                std::cerr << dir_itr->path().filename() << " " << ex.what() << std::endl;
            }
        }
    } else {
        yml_paths.push_back(path);
    }

    int zoom = 1;
    int raw  = 0;
    if(argc > 2) {
        zoom = atoi(argv[2]);
    }

    if(argc > 3) {
        raw = atoi(argv[3]);
    }

    /// up : 1113938 65362
    /// down : 1113940 65362
    /// esc  : 27 1048603

    if(yml_paths.size() == 0) {
        std::cerr << "No paths found !" << std::endl;
        return 1;
    }

    if(jpg_paths.size() == 0) {
        std::cout << "Single files doesn't support images or no images where found!" << std::endl;
    }

    std::sort(yml_paths.begin(), yml_paths.end());
    std::sort(jpg_paths.begin(), jpg_paths.end());

    cv::namedWindow("Generation");
    cv::Vec<int,5> update_vec(0,0,0,0,0);
    cv::setMouseCallback("Generation", mouse_event, &update_vec);
    bool end = false;
    int  yml_pos = 0;
    int  jpg_pos = 0;
    int  cell_height = zoom;
    int  cell_width  = zoom;
    bool use_jpg = jpg_paths.size() > 0;
    int filled = 0;
    while(!end) {
        TerraMat mat_original;
        cv::Mat image;
        cv::Mat render;
        std::string path_original = yml_paths[yml_pos];

        mat_original.read(path_original);

        TerraMat mat;
        mat_original.getAbsolut(mat);
        int channel_set = ((cv::Mat) mat).channels() + 1;

        if(use_jpg > 0) {
            image = cv::imread(jpg_paths[jpg_pos]);
        }

        cv::Mat canvas;
        if(image.empty())
            canvas = cv::Mat(((cv::Mat) mat).rows * cell_height, ((cv::Mat) mat).cols * cell_width , CV_8UC3, cv::Scalar::all(0));
        else {
            canvas = image.clone();
            cv::resize(canvas, canvas, cv::Size(((cv::Mat) mat).cols * cell_width, ((cv::Mat) mat).rows * cell_height));
        }

        if(!((cv::Mat) mat).empty()) {
            bool active_mat = true;
            while(active_mat) {
                if(!changes.empty()) {
                    size_t n_changes = changes.size();
                    for(size_t change_i = 0; change_i < n_changes; ++change_i) {
                        const cv::Vec<int,5>& change = changes[change_i];

                        int d = brush_width;

                        for(int dx=-d; dx <= d; ++dx) {
                            for(int dy=-d; dy <= d; ++dy) {
                                int grid_x = change[0] / cell_width + dx;
                                int grid_y = change[1] / cell_height + dy;

                                int cols = ((cv::Mat) mat).cols;
                                int rows = ((cv::Mat) mat).rows;
                                if(grid_x < 0 || grid_x >= cols || grid_y < 0 || grid_y >= rows)
                                    continue;

                                if(change[2] < channel_set) {
                                    mat.setAbsolut(grid_y, grid_x, change[2]);
                                } else {
                                    mat.setUnknown(grid_y, grid_x);
                                }
                            }
                        }

                        changes.clear();
                    }

                }

                cv::Mat bgr;
                if(raw == 0)
                    bgr = mat.getFavoritesBGR();
                else
                    bgr = mat.getFavoritesBGRRaw();

                canvas.copyTo(render);

                for(int i = 0 ; i < bgr.rows ; ++i) {
                    for(int j = 0 ; j < bgr.cols ; ++j) {
                        cv::Rect rect(j * cell_width, i * cell_height , cell_width, cell_height);
                        cv::Vec3b  bgr_val = bgr.at<cv::Vec3b>(i,j);
                        cv::Scalar color(bgr_val[0] , bgr_val[1], bgr_val[2]);
                        if(!image.empty()) {
                            if(filled == 0)
                                cv::rectangle(render, rect, color, 2);
                            if(filled == 1)
                                cv::rectangle(render, rect, color, CV_FILLED);
                        } else {
                            cv::rectangle(render, rect, color, CV_FILLED);
                        }
                    }
                }

                cv::imshow("Generation", render);
                int key = cv::waitKey(19);
                switch(key) {
                case 27:
                    end = true;
                    active_mat = false;
                    break;
                case 1048603:
                    end = true;
                    active_mat = false;
                    break;
                case 1113938:
                    --jpg_pos;
                    --yml_pos;
                    active_mat = false;
                    break;
                case 65362:
                    --jpg_pos;
                    --yml_pos;
                    active_mat = false;
                    break;
                case 1113940:
                    ++jpg_pos;
                    ++yml_pos;
                    active_mat = false;
                    break;
                case 65364:
                    active_mat = false;
                    ++jpg_pos;
                    ++yml_pos;
                    break;
                case 1048691:
                    mat.write(path_original);
                    break;
                case 115:
                    mat.write(path_original);
                    break;
                case 109:
                    filled = (filled + 1 ) % 3;
                    break;
                case 1048685:
                    filled = (filled + 1 ) % 3;
                    break;
                case 1048625:
                    update_vec[2]++;
                    break;
                case 49:
                    update_vec[2]++;
                    break;
                case 1048626:
                    update_vec[2]--;
                    break;
                case 50:
                    update_vec[2]--;
                    break;
                }
                update_vec[2] = (update_vec[2] + channel_set) % channel_set;
            }
        } else {
            render = cv::Mat(480,640, CV_8UC3);
        }
        yml_pos = (yml_pos + yml_paths.size()) % yml_paths.size();
        if(use_jpg)
            jpg_pos = (jpg_pos + jpg_paths.size()) % jpg_paths.size();

    }

    cv::destroyWindow("Generation");
    return 0;
}

