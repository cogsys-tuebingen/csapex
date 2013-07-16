#include "combiner_gridcompare_hist.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareHist, vision_evaluator::ImageCombiner)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareHist::GridCompareHist() :
    container_hist_sliders_(NULL)
{
    state_.channel_count = 0;
    state_.restored = false;
}

cv::Mat GridCompareHist::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");

        if(state_.channel_count != img1.channels()) {
            state_.channel_count = img1.channels();
            state_.bins.clear();
            state_.eps.clear();
            Q_EMIT modelChanged();
        }

        if(hist_sliders_.size() == state_.channel_count) {
            GridHist g1, g2;
            AttrHistogram::Params p;
            prepareHistParams(p.bins, p.ranges, p.eps);
            int index = combo_compare_->currentIndex();
            p.method = index_to_compare_[index];

            int width = slide_width_->value();
            int height = slide_height_->value();

            /// MEMENTO
            state_.combo_index = index;
            state_.grid_width = width;
            state_.grid_height = height;

            cv_grid::prepare_grid<AttrHistogram>(g1, img1, height, width, p, mask1, 1.0);
            cv_grid::prepare_grid<AttrHistogram>(g2, img2, height, width, p, mask2, 1.0);

            cv::Mat out(img1.rows + 40, img1.cols, CV_8UC3, cv::Scalar(0,0,0));
            render_grid(g1, g2, out);
            return out;
        }
    }
    return cv::Mat();
}

void GridCompareHist::updateGui(QBoxLayout *layout)
{
    QVBoxLayout *internal_layout;
    if(container_hist_sliders_ != NULL) {
        container_hist_sliders_->deleteLater();
    }
    internal_layout = new QVBoxLayout;

    for(int i = 0 ; i < state_.channel_count ; i++) {
        std::stringstream ch;
        ch << i + 1;

        int    default_bin = 32;
        double default_eps = 0.0;

        if(state_.restored) {
            default_bin = state_.bins[i];
            default_eps = state_.eps[i];
        } else {
            state_.bins.push_back(default_bin);
            state_.eps.push_back(default_eps);
        }

        QSlider *bins = QtHelper::makeSlider(internal_layout, "Ch." + ch.str() + " bins", default_bin, 1, 1000);
        QDoubleSlider *eps = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + " eps", default_eps, 0.0, 255.0, 0.01);
        insertSliders(bins, eps);
    }

    if(state_.restored) {
        slide_height_->setValue(state_.grid_height);
        slide_width_->setValue(state_.grid_width);
        combo_compare_->setCurrentIndex(state_.combo_index);
        state_.restored = false;
    }

    container_hist_sliders_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_hist_sliders_);


}

Memento::Ptr GridCompareHist::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void GridCompareHist::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());
    state_ = *m;
    Q_EMIT modelChanged();
}



void GridCompareHist::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
    state_.restored = false;

    combo_compare_ = new QComboBox();
    combo_compare_->addItem("Correlation");
    combo_compare_->addItem("Chi-Square");
    combo_compare_->addItem("Intersection");
    combo_compare_->addItem("Hellinger");

    int index = combo_compare_->findText("Correlation");
    index_to_compare_.insert(intPair(index, CV_COMP_CORREL));
    index = combo_compare_->findText("Chi-Square");
    index_to_compare_.insert(intPair(index, CV_COMP_CHISQR));
    index = combo_compare_->findText("Intersection");
    index_to_compare_.insert(intPair(index, CV_COMP_INTERSECT));
    index = combo_compare_->findText("Hellinger");
    index_to_compare_.insert(intPair(index, CV_COMP_BHATTACHARYYA));

    layout->addWidget(combo_compare_);
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
    bins = cv::Mat_<int>(state_.channel_count, 1);
    ranges = cv::Mat_<float>(state_.channel_count * 2 ,1);
    for(int i = 0 ; i < state_.channel_count ; i++) {
        HistSliderPair p = hist_sliders_[i];
        bins.at<int>(i)     = p.first->value();
        ranges.at<float>(2 * i)     = 0.f;
        ranges.at<float>(2 * i + 1) = 256.f;
        eps[i]          = p.second->doubleValue();

        /// MEMENTO
        state_.bins[i] = p.first->value();
        state_.eps[i] = p.second->doubleValue();
    }
}
