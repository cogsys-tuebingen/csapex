#ifndef ROC_CREATOR_H
#define ROC_CREATOR_H

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/signals2.hpp>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief The RocCreator class creates a dynamic ROC-Diagram
 */
class RocCreator
{
public:
    /**
     * @brief The Data class is used to represents Points on the ROC curve
     */
    class Data
    {
    public:
        double score;
        double dtheta;
        double dr;
        cv::Mat user_data;

        Data()
            : score(INFINITY), dtheta(0), dr(0)
        {}

        bool operator < (const Data& rhs) const {
            return (score < rhs.score);
        }
    };

    struct L_data {
        Data data;
        bool positive;
    };

    struct R_data {
        const L_data* data;
        cv::Point2d pt;
        bool positive;
        bool meta;
    };



public:
    /**
     * @brief RocCreator
     * @param width width of the drawing area
     * @param padding additional spacing
     * @param title
     */
    RocCreator(int width = 0, int padding = 0, const std::string& title = "");

    /**
     * @brief setTitle
     * @param title
     */
    void setTitle(const std::string& title);

    /**
     * @brief save saves the diagram to a path
     * @param path
     */
    void save(const std::string& path);

    /**
     * @brief load loads a diagram from a path
     * @param path
     * @return
     */
    bool load(const std::string& path);

    /**
     * @brief displayInteractive displays the ROC graph and allow user interaction
     */
    void displayInteractive();

    /**
     * @brief stop stops the interactive mode
     */
    void stop();

    /**
     * @brief add adds a data item
     * @param data
     * @param is_positive_example
     * @return false, iff shutdown is requested
     */
    bool add(const Data& data, bool is_positive_example);

    /**
     * @brief getErrorsForThreshold analyzes the curve for all entries below a threshold
     * @param threshold the maximum threshold to evaluate
     * @param out outstream to write to
     * @param tp Output: true-positive rate
     * @param fp Output: false-positive rate
     */
    void getErrorsForThreshold(double threshold, std::vector<double> &out, double& tp, double& fp);

public:
    /**
     * @brief score_selected signals where the user has clicked into the diagram
     */
    boost::signals2::signal<void (RocCreator* sender, const Data* selected, bool save)> score_selected;

private:
    void initImage();
    bool draw();

private:
    int p;
    int n;

    int fp;
    int tp;

    int dimension_raw;

    int w;
    int padding;

    cv::Mat roc;

    std::vector<R_data> R;
    std::vector<L_data> L;

    bool keep_running;

    std::string title;

    int zoom;
    int zoom_max;
};

#endif // ROC_CREATOR_H
