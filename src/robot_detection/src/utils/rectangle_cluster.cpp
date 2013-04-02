/// HEADER
#include "rectangle_cluster.h"

namespace
{
std::ostream& operator << (std::ostream& out, cv::Rect& r)
{
    out << '[' << r.tl() << " " << r.br() << ']';
    return out;
}
}

RectangleCluster::RectangleCluster()
{
}


bool RectangleCluster::contains(const cv::Rect& test)
{
    return std::find(rois.begin(), rois.end(), test) != rois.end();
}

bool RectangleCluster::covers(const cv::Rect& test)
{
    for(std::vector<cv::Rect>::iterator it = rois.begin(); it != rois.end(); ++it) {
        cv::Rect& roi = *it;

        if(roi == test || roi == (roi | test)) {
            /// rec is equal to a roi
            ///  or contained in a roi
            return true;
        }
    }

    return false;
}

bool RectangleCluster::conflicts(const cv::Rect& test)
{
    for(std::vector<cv::Rect>::iterator it = rois.begin(); it != rois.end(); ++it) {
        cv::Rect& roi = *it;

        if(test != roi) {
            cv::Rect inter = roi & test;
            if(inter.width > 0 && inter.height > 0) {
                return true;
            }
        }
    }

    return false;
}

void RectangleCluster::integrate(const cv::Rect& rec)
{
    /// invariant: no two rois are overlapping
    if(covers(rec)) {
        return;
    }

    /// invariant: no two rois are overlapping
    /// rec is not in rois
    /// rec is not fully contained in one roi

    for(std::vector<cv::Rect>::iterator it = rois.begin(); it != rois.end(); ++it) {
        cv::Rect& roi = *it;

        cv::Rect inter = roi & rec;
        if(inter.width == 0 || inter.height == 0) {
            continue;
        }

        /// invariant: no two rois are overlapping
        /// rec is not in rois
        /// rec is not fully contained in one roi
        /// rec intersects roi

        cv::Rect merged = roi | rec;
        if(!conflicts(merged)) {
            /// invariant: no two rois are overlapping
            /// rec is not in rois
            /// rec is not fully contained in one roi
            /// merged is not intersecting any roi

            roi = merged;

            /// invariant: no two rois are overlapping
            return;

        } else {
            /// invariant: no two rois are overlapping
            /// rec is not in rois
            /// rec is not fully contained in one roi
            /// merged is intersecting some roi

            rois.erase(it);

            integrate(merged);

            return;
        }
    }

    /// invariant: no two rois are overlapping
    /// rec is not in rois
    /// rec is not fully contained in one roi
    /// rec is not intersecting any roi

    rois.push_back(rec);

    /// invariant: no two rois are overlapping
}
