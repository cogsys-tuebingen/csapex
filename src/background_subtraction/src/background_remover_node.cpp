/// HEADER
#include "background_remover_node.h"

/// PROJECT
#include <utils/LibUtil/QtCvImageConverter.h>

/// SYSTEM
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <utils/LibUtil/Stopwatch.h>

using namespace background_subtraction;

BackgroundRemoverNode::BackgroundRemoverNode(ros::NodeHandle& nh, QMutex& img_mutex)
    : n(nh), img_mutex(img_mutex), has_images(false), use_frame_as_bg(true)
{
}

BackgroundRemoverNode::~BackgroundRemoverNode()
{
    wait();
}

bool BackgroundRemoverNode::init()
{
    if ( ! ros::master::check() ) {
        return false;
    }

    std::string filtered = "/camera/image_filtered";
    std::string masked = "/camera/image_masked";
    n.param("filtered_topic", filtered, filtered);
    n.param("masked_topic", masked, masked);

    image_sub = n.subscribe<sensor_msgs::Image>
                ("/camera/image_raw", 5, boost::bind(&BackgroundRemoverNode::imageCB, this, _1));
    filtered_pub = n.advertise<sensor_msgs::Image> (filtered, 4);
    masked_pub = n.advertise<background_subtraction::MaskedImage> (masked, 4);


    for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin();
        it != BackgroundRemover::metaInstance().instances.end();
        ++it) {
        (*it)->init();
    }

    active_background_remover = BackgroundRemover::metaInstance().instances[0];

    f = boost::bind(&BackgroundRemoverNode::dynamicReconfigureCB, this, _1, _2);
    server.setCallback(f);

    start();
    return true;
}

void BackgroundRemoverNode::applyConfig(background_subtraction::GlobalConfig& config)
{
    current_config = config;

    changeAlgo(config.algorithm);
    setThreshold(config.threshold);
    setOpen(config.open);
    setClose(config.close);


    for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin(); it != BackgroundRemover::metaInstance().instances.end(); ++it) {
        (*it)->applyConfig(config);
    }
}

void BackgroundRemoverNode::dynamicReconfigureCB(background_subtraction::GlobalConfig& config, uint32_t level)
{
    applyConfig(config);

    Q_EMIT configChanged();
}

void BackgroundRemoverNode::update(background_subtraction::GlobalConfig& config)
{
    applyConfig(config);
    server.updateConfig(config);
}

void BackgroundRemoverNode::setBackground()
{
    use_frame_as_bg = true;
}

void BackgroundRemoverNode::changeAlgo(int i)
{
    int n = BackgroundRemover::metaInstance().instances.size();
    if(i >= n) {
        std::cout << "unknown algorithm selected: " << i << " of " << n << "!" << std::endl;
        assert(false);
    }

    active_background_remover = BackgroundRemover::metaInstance().instances[i];
}

void BackgroundRemoverNode::setThreshold(int i)
{
    for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin(); it != BackgroundRemover::metaInstance().instances.end(); ++it) {
        (*it)->setThreshold(i);
    }
}

void BackgroundRemoverNode::setOpen(int i)
{
    for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin(); it != BackgroundRemover::metaInstance().instances.end(); ++it) {
        (*it)->setOpen(i);
    }
}

void BackgroundRemoverNode::setClose(int i)
{

    for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin(); it != BackgroundRemover::metaInstance().instances.end(); ++it) {
        (*it)->setClose(i);
    }
}

void BackgroundRemoverNode::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    Stopwatch watch;

    /// convert from sensor image to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /// perform filtering
    cv::Mat filtered, mask, bg, debug;
    cv::Mat& frame = cv_ptr->image;

    /// initialize background remover if necessary
    if(use_frame_as_bg) {
        use_frame_as_bg = false;


        for(std::vector<BackgroundRemover*>::iterator it = BackgroundRemover::metaInstance().instances.begin(); it != BackgroundRemover::metaInstance().instances.end(); ++it) {
            (*it)->setBackground(frame);
        }
    }

    /// filter the background
    if(active_background_remover->ready()) {
        active_background_remover->filter(frame, filtered, mask);
        bg = active_background_remover->getBackground();
    } else {
        filtered = cv::Mat::zeros(50, 50, CV_8U);
        mask = cv::Mat::zeros(50, 50, CV_8U);
        bg = cv::Mat::zeros(50, 50, CV_8U);
    }

    debug = active_background_remover->getDebugImage();

    /// publish results
    masked_pub.publish(masked_img_handler.wrap(msg->header, msg, mask));

    cv_bridge::CvImage filtered_converter;
    filtered_converter.encoding = msg->encoding;
    filtered_converter.header = msg->header;
    filtered_converter.image = filtered;

    filtered_pub.publish(filtered_converter.toImageMsg());

    int ms = watch.msElapsed();
    std::cout << "time for processing: " << ms << "ms = " << (1000.0 / ms) << " fps" << std::endl;

    /// update qimages for visualization
    QMutexLocker lock(&img_mutex);

    has_images = false;

    double scale = 200.0 / frame.rows;

    cv::resize(frame, frame, cv::Size(), scale, scale);

    if(active_background_remover->ready()) {
        cv::resize(filtered, filtered, cv::Size(), scale, scale);
        cv::resize(mask, mask, cv::Size(), scale, scale);
        cv::resize(bg, bg, cv::Size(), scale, scale);
        cv::resize(debug, debug, cv::Size(), scale, scale);
    }

    frame_ = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(frame);
    filtered_ = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(filtered);
    mask_ = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mask);
    bg_image_ = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(bg);
    debug_ = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(debug);

    has_images = true;

    Q_EMIT imageChanged();
}

void BackgroundRemoverNode::run()
{
    ros::Rate loop_rate(60);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
