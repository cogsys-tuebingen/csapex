#ifndef CONFIG_H
#define CONFIG_H

/// COMPONENT
#include "types.h"

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/serialization/split_member.hpp>
#include <boost/signals2.hpp>

/**
 * @brief The Config class extends the automatically generated ROS config
 *        and contains settings for every part of the program
 */
class Config
{
    friend class boost::serialization::access;
    friend class Reconfigurable;

protected:
    /**
     * @brief Config
     */
    Config();

public:
    /**
     * @brief applyGlobal replaces all instances of Config with the one the method is called on
     */
    void replaceGlobal() const;

    /**
     * @brief get Accessor
     */
    static Config getGlobal();

    /**
     * @brief getDescription returns a descriptive name of this config
     * @return a descriptive name
     */
    std::string getDescription() const;

    /**
     * @brief computeHash returns a hash code for this config
     * @return a hash code
     */
    std::string computeHash() const;

protected:
    static Config latest;

    static boost::signals2::signal<void(const Config&)> replace;

public:
    Types::Strategy::ID db_type;

    std::string db_file;
    std::string db_imgs;
    int bin_count;
    int bin_max_poses_count;
    int bin_min_feature_count;

    std::string config_dir;
    std::string result_dir;
    std::string batch_dir;
    std::string ref_dir;
    std::string name;

    bool gui_enabled;

public: // these came originally from ros
    std::string keypoint_name;
    std::string descriptor_name;
    double angle_offset;
    double sample_threshold;
    double score_threshold;
    int octaves;
    int extractor_threshold;
    double matcher_threshold;
    int min_points;
    bool use_pruning;
    bool crop_test;
    bool interactive;

public:
    const std::string& getKeypointType() const;
    const std::string& getDescriptorType() const;

    void setKeypointType(const std::string& type);
    void setDescriptorType(const std::string& type);

protected:
    virtual void make_defaults();
    void init();

protected:
    template<class Archive>
    void serialize(Archive& ar, const unsigned int file_version) {
        //boost::serialization::split_member(ar, *this, file_version);

        ar& db_file;
        ar& db_type;
        ar& bin_count;
        ar& bin_max_poses_count;
        ar& bin_min_feature_count;
        ar& config_dir;
        ar& result_dir;
        ar& batch_dir;

        ar& keypoint_name;
        ar& descriptor_name;
        ar& angle_offset;
        ar& sample_threshold;
        ar& score_threshold;
        ar& octaves;
        ar& extractor_threshold;
        ar& matcher_threshold;
        ar& min_points;
        ar& use_pruning;
        ar& crop_test;
        ar& interactive;

        setKeypointType(keypoint_name);
        setDescriptorType(descriptor_name);
    }

//    template<class Archive>
//    void save(Archive& ar, const unsigned int version) const {
//        ar& db_file;
//        ar& db_type;
//        ar& bin_count;

//        ar& config_dir;
//        ar& result_dir;
//        ar& batch_dir;
//    }

//    template<class Archive>
//    void load(Archive& ar, const unsigned int version) {
//        ar& db_file;
//        ar& db_type;
//        ar& bin_count;

//        ar& config_dir;
//        ar& result_dir;
//        ar& batch_dir;
//    }
};

#endif // CONFIG_H
