#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <utils/LibCvTools/random_generator.hpp>
#include <vector>
#include <set>

namespace files {

inline bool create_on_check(const std::string &path)
{
    if(!boost::filesystem3::exists(path)) {
        std::cout << "Directory is not existing, therefore creating it as sub dir!" << std::endl;
        if(!boost::filesystem3::create_directories(path)) {
            std::cerr << "Cannot create directory '" << path << "' !" << std::endl;
            return false;
        }
    }
    return true;
}

inline bool read_yml_img(const std::string &path, std::vector<std::string> &imgs, std::vector<std::string> &rois, std::vector<std::string> &ymls)
{
    boost::regex e_yaml(".*\\.(yml)");
    boost::regex e_jpg(".*\\.(jpg)");
    boost::regex e_roi(".*\\.(rois\\.yaml)");
    if(boost::filesystem3::is_directory(path)) {
        boost::filesystem3::directory_iterator end_iter;
        for(boost::filesystem3::directory_iterator dir_itr( path ); dir_itr != end_iter; ++dir_itr) {
            try {
                boost::cmatch what;
                if (boost::filesystem3::is_regular_file( dir_itr->status())) {
                    std::string current = dir_itr->path().string();
                    if(boost::regex_match(current.c_str(), what, e_yaml) && what[0].matched) {
                        ymls.push_back(current);
                    }
                    if(boost::regex_match(current.c_str(), what, e_jpg) && what[0].matched) {
                        imgs.push_back(current);
                    }
                    if(boost::regex_match(current.c_str(), what, e_roi) && what[0].matched) {
                        rois.push_back(current);
                    }
                }
            }
            catch ( const std::exception & ex ){
                std::cerr << dir_itr->path().filename() << " " << ex.what() << std::endl;
            }
        }
    }

    return !imgs.empty() && imgs.size() == ymls.size() && imgs.size() == rois.size();
}

}

int main(int argc, char *argv[])
{
    std::string path = boost::filesystem3::current_path().string();

    if(argc < 3 ) {
        std::cout << "Arguments : [<src path>] <train set> <validation set>" << std::endl;
        return 1;
    }

    std::string path_train = argv[1];
    std::string path_valid = argv[2];
    if(argc == 4) {
        path        = argv[1];
        path_train  = argv[2];
        path_valid  = argv[3];
    }

    if(path_train == "" || path_valid == "") {
        std::cerr << "Path length was 0 !" << std::endl;
        return 1;
    }

    if(path_train[path_train.length() - 1] != '/')
        path_train += '/';

    if(path_valid[path_valid.length() - 1] != '/')
        path_valid += '/';

    if(!files::create_on_check(path_train))
        return 1;
    if(!files::create_on_check(path_valid))
        return 1;

    std::vector<std::string> img_paths;
    std::vector<std::string> yml_paths;
    std::vector<std::string> roi_paths;
    if(!files::read_yml_img(path, img_paths, roi_paths, yml_paths) || yml_paths.empty() || img_paths.empty() || roi_paths.empty()) {
        std::cerr << "Directory not readable, no files given or the amount of .yml and .img files is not matching!" << std::endl;
        if(yml_paths.empty()) {
            std::cerr << "No yml paths found !" << std::endl;
        }

        if(img_paths.empty()) {
            std::cout << "No images where found!" << std::endl;
        }

        if(roi_paths.empty())
            std::cout << "No rois where found!" << std::endl;

        return 1;
    }

    std::sort(yml_paths.begin(), yml_paths.end());
    std::sort(img_paths.begin(), img_paths.end());
    std::sort(roi_paths.begin(), roi_paths.end());

    /// SAMPLE 50% TRAIN DATA AND 50% VALIDATION DATA

    std::set<int>  sample_indeces;
    int sample_elements = .5 * img_paths.size();
    RandomGeneratorInt         generator(0, img_paths.size() - 1);
    while(sample_indeces.size() < sample_elements) {
        int index = generator.generate();
        sample_indeces.insert(index);
    }


    std::vector<std::string> train_images;
    std::vector<std::string> train_rois;
    for(std::set<int>::iterator it = sample_indeces.begin() ; it != sample_indeces.end() ; ++it) {
        train_images.push_back(img_paths[*it]);
        train_rois.push_back(roi_paths[*it]);
    }

    for(std::set<int>::iterator it = sample_indeces.begin() ; it != sample_indeces.end() ; ++it) {
        img_paths.erase(img_paths.begin() + *it);
        yml_paths.erase(yml_paths.begin() + *it);
    }


    /// DO THE COPY STUFF
    /// TRAIN
    for(int i = 0 ; i < train_images.size() ; ++i) {
        boost::filesystem3::path src_img(train_images[i]);
        boost::filesystem3::path dst_img(path_train + src_img.filename().string());
        boost::filesystem3::path src_roi(train_rois[i]);
        boost::filesystem3::path dst_roi(path_train + src_roi.filename().string());
        boost::filesystem3::copy_file(src_img, dst_img, boost::filesystem3::copy_option::overwrite_if_exists);
        boost::filesystem3::copy_file(src_roi, dst_roi, boost::filesystem3::copy_option::overwrite_if_exists);
    }

    /// VALIDATION
    for(int i = 0 ; i < img_paths.size() ; ++i) {
        boost::filesystem3::path src_img(img_paths[i]);
        boost::filesystem3::path src_yml(yml_paths[i]);
        boost::filesystem3::path dst_img(path_valid + src_img.filename().string());
        boost::filesystem3::path dst_yml(path_valid + "gt_" + src_yml.filename().string());
        boost::filesystem3::copy_file(src_img, dst_img, boost::filesystem3::copy_option::overwrite_if_exists);
        boost::filesystem3::copy_file(src_yml, dst_yml, boost::filesystem3::copy_option::overwrite_if_exists);

    }

    std::cout << "Sets were generated!" << std::endl;
    return 0;
}

