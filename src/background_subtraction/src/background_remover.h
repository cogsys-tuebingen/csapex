#ifndef BACKGROUND_REMOVER_H
#define BACKGROUND_REMOVER_H

/// PROJECT
#include <background_subtraction/GlobalConfig.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

#define REGISTER_REMOVER(classname)\
    class classname##registerer {\
        classname##registerer() {\
            std::cout << "registering " << #classname << std::endl;\
            background_subtraction::BackgroundRemover* i = new classname();\
            background_subtraction::BackgroundRemover::metaInstance().instances.push_back(i);\
        }\
        static classname##registerer reg;\
    };\
    classname##registerer classname##registerer::reg



namespace background_subtraction
{

/**
 * @brief The BackgroundRemover class the base class for different classes used to mask the background in an image.
 */
class BackgroundRemover
{
public:
    struct Meta {
        std::vector<BackgroundRemover*> instances;
    };

    static Meta& metaInstance() {
        static Meta m;
        return m;
    }

public:
    /**
     * @brief BackgroundRemover
     * @param name the name of this remover
     */
    BackgroundRemover(const std::string& name);

    /**
     * @brief ~BackgroundRemover
     */
    virtual ~BackgroundRemover();

    /**
     * @brief init initialized to remover, must be called once before filter is used
     */
    virtual void init();

    /**
     * @brief filter Takes an images and produces a filtered image and a mask
     * @param frame Input image
     * @param filtered Output: filtered image
     * @param mask Output: mask
     */
    void filter(const cv::Mat& frame, cv::Mat& filtered, cv::Mat& mask);

    /**
     * @brief ready
     * @return true, iff the remover is ready to process images
     */
    virtual bool ready() {
        return has_background;
    }

    /**
     * @brief getDebugImage default: not implemented
     * @return an empty image
     */
    virtual cv::Mat getDebugImage() {
        return NO_DEBUG;
    }

    /**
     * @brief setBackground Setter
     * @param frame The frame to use as background
     */
    virtual void setBackground(const cv::Mat& frame);

    /**
     * @brief getBackground Accessor
     * @return background image
     */
    virtual cv::Mat getBackground() {
        return background;
    }

    /**
     * @brief getName
     * @return the name of this remover
     */
    const std::string& getName() {
        return name_;
    }

    /**
     * @brief applyConfig
     * @param config
     */
    virtual void applyConfig(background_subtraction::GlobalConfig& config) {
        difference_threshold = config.threshold;
        open_iterations = config.open;
        close_iterations = config.close;
        triggerChanged();
    };

    /**
     * @brief setThreshold Setter
     * @param i the difference threshold
     */
    void setThreshold(int i) {
        difference_threshold = i;
        triggerChanged();
    }
    /**
     * @brief getThreshold Accessor
     * @return the difference threshold
     */
    int getThreshold() {
        return difference_threshold;
    }

    /**
     * @brief setOpen Setter
     * @param i the number of open iterations
     */
    void setOpen(int i) {
        open_iterations = i;
        triggerChanged();
    }
    /**
     * @brief getOpen Getter
     * @return the number of open iterations
     */
    int getOpen() {
        return open_iterations;
    }

    /**
     * @brief setClose Setter
     * @param i the numer of close iterations
     */
    void setClose(int i) {
        close_iterations = i;
        triggerChanged();
    }
    /**
     * @brief getClose Accessor
     * @return the numer of close iterations
     */
    int getClose() {
        return close_iterations;
    }

protected:
    /**
     * @brief triggerChanged callback for when config has changed
     */
    void triggerChanged() {
        changed = true;
    }

protected: // abstract
    /**
     * @brief segmentation Template method for subclasses
     * @param original the current frame
     * @param mask Output: the generated mask
     */
    virtual void segmentation(const cv::Mat& original, cv::Mat& mask) = 0;

private:
    void createResult(cv::Mat& filtered, cv::Mat frame_blured, cv::Mat mask);

private:
    static cv::Mat NO_DEBUG;

protected:
    std::string name_;
    bool has_background;
    cv::Mat background;
    cv::Mat background_blured;

    int difference_threshold;

protected:
    double sigma_blur;
    cv::Size blur_kernel;

    int close_iterations;
    int open_iterations;

    bool changed;
};

}
#endif // BACKGROUND_REMOVER_H
