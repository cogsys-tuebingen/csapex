#ifndef BACKGROUND_REMOVER_NODE_H
#define BACKGROUND_REMOVER_NODE_H

/// COMPONENT
#include "adaptive_background_remover.h"
#include "background_remover.h"
#include "simple_background_remover.h"
#include "vibe_background_remover.h"
#include "opencv_background_remover.h"

/// PROJECT
#include <background_subtraction/masked_image_handler.h>
#include <background_subtraction/GlobalConfig.h>

/// SYSTEM
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <QImage>
#include <QMutex>
#include <QThread>
#include <QSharedPointer>

namespace background_subtraction
{

/**
 * @brief The BackgroundRemoverNode class is an Adapter for BackgroundRemover and connects it to ROS and QT
 */
class BackgroundRemoverNode : public QThread
{
    Q_OBJECT

    friend class BackgroundRemoverWindow;

    typedef QSharedPointer<QImage> QImagePtr;

public:
    /**
     * @brief BackgroundRemoverNode
     * @param nh ROS NodeHandle
     * @param img_mutex Mutex for synchronization
     */
    BackgroundRemoverNode(ros::NodeHandle& nh, QMutex& img_mutex);

    /**
     * @brief ~BackgroundRemoverNode
     */
    virtual ~BackgroundRemoverNode();

    /**
     * @brief init initializes everything
     * @return true, iff everything could be intialized
     */
    bool init();

    /**
     * @brief run loops until shutdown is requested
     */
    void run();

    /**
     * @brief imageCB Callback function. Is called when a new Image has arrived
     * @param msg The image in a ROS message format
     */
    void imageCB(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief dynamicReconfigureCB Callback function. Is called when a reconfiguration request has arrived
     * @param config the config to use
     * @param level
     */
    void dynamicReconfigureCB(background_subtraction::GlobalConfig& config, uint32_t level);

    /**
     * @brief update  Callback function. Is called when a reconfiguration request has arrived
     * @param config the config to use
     */
    void update(background_subtraction::GlobalConfig& config);

    /**
     * @brief getCurrentConfig Accessor
     * @return the current configuration
     */
    background_subtraction::GlobalConfig getCurrentConfig() const {
        return current_config;
    }

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void imageChanged();
    void configChanged();

public Q_SLOTS:
    void setBackground();

private:
    void applyConfig(background_subtraction::GlobalConfig& config);

    void changeAlgo(int i);
    void setThreshold(int i);
    void setOpen(int i);
    void setClose(int i);

private:
    ros::NodeHandle& n;

    QMutex& img_mutex;

    ros::Subscriber image_sub;
    ros::Publisher filtered_pub;
    ros::Publisher masked_pub;

    MaskedImageHandler masked_img_handler;

    dynamic_reconfigure::Server<background_subtraction::GlobalConfig> server;
    dynamic_reconfigure::Server<background_subtraction::GlobalConfig>::CallbackType f;
    background_subtraction::GlobalConfig current_config;

    BackgroundRemover* active_background_remover;
    VibeBackgroundRemover vibe_remover;
    SimpleBackgroundRemover simple_remover;
    AdaptiveBackgroundRemover adaptive_remover;
    OpenCvBackgroundRemover cv_remover;

    bool has_images;
    bool use_frame_as_bg;

    QImagePtr frame_;
    QImagePtr mask_;
    QImagePtr filtered_;
    QImagePtr bg_image_;
    QImagePtr debug_;

};

}

#endif // BACKGROUND_REMOVER_NODE_H
