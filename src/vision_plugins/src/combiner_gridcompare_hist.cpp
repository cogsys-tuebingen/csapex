#include "combiner_gridcompare_hist.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareHist, vision_evaluator::ImageCombiner)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareHist::GridCompareHist() :
    channel_count_(0),
    container_hist_sliders_(NULL)
{
}

cv::Mat GridCompareHist::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        assert(img1.channels() == img2.channels());

        if(channel_count_ != img1.channels()) {
            channel_count_ = img1.channels();
            updateSliders();
        }

        GridHist g1, g2;
        AttrHistogram::Params p;
        prepareHistParams(p.bins, p.ranges, p.eps);
        p.method = index_to_compare_[combo_compare_->currentIndex()];

        int width = slide_width_->value();
        int height = slide_height_->value();

        cv_grid::prepare_grid<AttrHistogram>(g1, img1, height, width, p, mask1, 1.0);
        cv_grid::prepare_grid<AttrHistogram>(g2, img2, height, width, p, mask2, 1.0);

        cv::Mat out(img1.rows + 20, img1.cols, CV_8UC3, cv::Scalar(0,0,0));
        intPair counts;
        int     valid;
        render_grid(g1, g2, out, counts, valid);

        std::stringstream text;
        text << "+" << counts.first << " | -" << counts.second << " | " << valid << std::endl;
        cv::putText(out, text.str(), cv::Point(0, out.rows - 2), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));

        return out;
    }
    return cv::Mat();
}

void GridCompareHist::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
    layout_ = layout;

    combo_compare_ = new QComboBox();
    combo_compare_->addItem("Correlation");
    combo_compare_->addItem("Chi-Square");
    combo_compare_->addItem("Intersection");
    combo_compare_->addItem("Hellinger");

    int index = combo_compare_->findText("Correlation");
    index_to_compare_.insert(intPair(index, CV_COMP_CORREL));
    compare_to_index_.insert(intPair(CV_COMP_CORREL, index));
    index = combo_compare_->findText("Chi-Square");
    index_to_compare_.insert(intPair(index, CV_COMP_CHISQR));
    compare_to_index_.insert(intPair(CV_COMP_CHISQR, index));
    index = combo_compare_->findText("Intersection");
    index_to_compare_.insert(intPair(index, CV_COMP_INTERSECT));
    compare_to_index_.insert(intPair(CV_COMP_INTERSECT, index));
    index = combo_compare_->findText("Hellinger");
    index_to_compare_.insert(intPair(index, CV_COMP_BHATTACHARYYA));
    compare_to_index_.insert(intPair(CV_COMP_BHATTACHARYYA, index));

    layout_->addWidget(combo_compare_);
}

void GridCompareHist::updateSliders()
{
    hist_sliders_.clear();
    QVBoxLayout *internal_layout;
    if(container_hist_sliders_ != NULL) {
        container_hist_sliders_->deleteLater();
    }
    internal_layout = new QVBoxLayout;

    for(int i = 0 ; i < channel_count_ ; i++) {
        std::stringstream ch;
        ch << i + 1;
        QSlider *bins = QtHelper::makeSlider(internal_layout, "Ch." + ch.str() + " bins", 32, 0, 1000);
        QDoubleSlider *eps = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + " eps", 0.0, 0.0, 255.0, 0.01);
        insertSliders(bins, eps);
    }

    container_hist_sliders_ = QtHelper::wrapLayout(internal_layout);
    layout_->addWidget(container_hist_sliders_);
}

void GridCompareHist::insertSliders(QSlider *bins, QDoubleSlider *eps)
{
    HistSliderPair p;
    p.first = bins;
    p.second = eps;
    hist_sliders_.push_back(p);
}

void GridCompareHist::prepareHistParams(cv::Mat &bins, cv::Mat &ranges, cv::Scalar &eps)
{
    bins = cv::Mat_<int>(channel_count_, 1);
    ranges = cv::Mat_<float>(channel_count_ * 2 ,1);
    for(int i = 0 ; i < channel_count_ ; i++) {
        HistSliderPair p = hist_sliders_[i];
        bins.at<int>(i)     = p.first->value();
        ranges.at<float>(2 * i)     = 0.f;
        ranges.at<float>(2 * i + 1) = 256.f;
        eps[i]          = p.second->value();

    }
}
