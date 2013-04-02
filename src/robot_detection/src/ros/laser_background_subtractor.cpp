/// HEADER
#include "laser_background_subtractor.h"

/// SYSTEM
#include <numeric>

LaserBackgroundSubtractor::LaserBackgroundSubtractor(const ros::NodeHandle& nh)
    : last_scan_dist(0)
{
    nh.param("laser_background_subtractor/dimension", dim, 10.0);
    nh.param("laser_background_subtractor/resolution", res, 0.05);
    nh.param("laser_background_subtractor/takes", takes_max, 5);

    takes = takes_max;

    pixels = ceil(dim / res);

    map = cv::Mat(pixels, pixels, CV_8U, cv::Scalar(0));

    cx = pixels / 2;
    cy = pixels / 2;
}

void LaserBackgroundSubtractor::process(const sensor_msgs::LaserScanConstPtr& msg, cv::Mat& out_visualization)
{
    if(takes > 0) {
        takes--;
        updateMap(msg);
        map.copyTo(out_visualization);

    } else {
        calcDist(msg).copyTo(out_visualization);
    }
}

void LaserBackgroundSubtractor::updateMap(const sensor_msgs::LaserScanConstPtr& msg)
{
    double angle = msg->angle_min;
    unsigned n = msg->ranges.size();
    for(unsigned i = 0; i < n; ++i) {
        double r = msg->ranges[i];
        double x = r * cos(angle);
        double y = r * sin(angle);

        int x_m = cx + x / res;
        int y_m = cy + y / res;

        angle += msg->angle_increment;

        if(x_m < 0 || y_m < 0 || x_m > pixels || y_m > pixels) {
            continue;
        }

        cv::circle(map, cv::Point(x_m, y_m), ceil(0.5 / res), cv::Scalar(255), CV_FILLED);
    }
}

cv::Mat LaserBackgroundSubtractor::calcDist(const sensor_msgs::LaserScanConstPtr& msg)
{
    cv::Mat map_tmp;

    cv::cvtColor(map, map_tmp, CV_GRAY2BGR);

    std::vector<double> xs;
    std::vector<double> ys;

    double angle = msg->angle_min;
    unsigned n = msg->ranges.size();
    for(unsigned i = 0; i < n; ++i) {
        double r = msg->ranges[i];
        double x = r * cos(angle);
        double y = r * sin(angle);

        int x_m = cx + x / res;
        int y_m = cy + y / res;

        angle += msg->angle_increment;

        if(x_m < 0 || y_m < 0 || x_m >= pixels || y_m >= pixels) {
            continue;
        }

        map_tmp.at<cv::Vec3b>(y_m, x_m) = cv::Vec3d(255, 0, 0);
        if(map.at<unsigned char>(y_m, x_m) != 255) {
            cv::circle(map_tmp, cv::Point(x_m, y_m), ceil(5.0 * res), cv::Scalar(0, 0, 255), CV_FILLED);

            xs.push_back(x);
            ys.push_back(y);
        }
    }

    if(!xs.empty() && !ys.empty()) {
        double mx = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
        double my = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();

        double x_sq_sum = std::inner_product(xs.begin(), xs.end(), xs.begin(), 0.0);
        double y_sq_sum = std::inner_product(ys.begin(), ys.end(), ys.begin(), 0.0);
        double x_stdev = std::sqrt(x_sq_sum / xs.size() - mx*mx);
        double y_stdev = std::sqrt(y_sq_sum / ys.size() - my*my);

        if(x_stdev > 1.0 || y_stdev > 1.0) {
            last_scan_dist = -1;
        } else {
            cv::circle(map_tmp, cv::Point(cx + mx / res, cy + my / res), 20, cv::Scalar(255, 255, 0), 2, CV_AA);

            last_scan_dist = std::sqrt(mx*mx + my*my);
        }

    } else {
        last_scan_dist = -1;
    }

    return map_tmp;
}


bool LaserBackgroundSubtractor::ready()
{
    return takes == 0;
}

double LaserBackgroundSubtractor::dist()
{
    return last_scan_dist;
}
