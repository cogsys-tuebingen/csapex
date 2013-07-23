#include "combiner_gridcompare_hist.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareHist, vision_evaluator::BoxedObject)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareHist::GridCompareHist() :
    GridCompare(State::Ptr(new State)),
    container_hist_sliders_(NULL)
{
    private_state_gch_ = dynamic_cast<State*>(state_.get());
    assert(private_state_gch_);
}

GridCompareHist::GridCompareHist(GridCompare::State::Ptr state) :
    GridCompare(state),
    container_hist_sliders_(NULL)
{
    private_state_gch_ = dynamic_cast<State*>(state_.get());
    assert(private_state_gch_);
}

cv::Mat GridCompareHist::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        /// PREPARE
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");
        //        if(img1.rows != img2.rows || img1.cols != img2.cols)
        //            throw std::runtime_error("Dimension is not matching!");


        if(private_state_gch_->channel_count != img1.channels()) {
            private_state_gch_->channel_count = img1.channels();
            private_state_gch_->bins.clear();
            private_state_gch_->eps.clear();
            Q_EMIT modelChanged();
        }

        updateSliderMaxima(img1.cols, img1.rows);

        /// COMPUTE
        if(hist_sliders_.size() == private_state_gch_->channel_count) {
            GridHist g1, g2;
            prepareGrid(g1, img1, mask1, private_state_gch_->grid_width, private_state_gch_->grid_width);
            prepareGrid(g2, img2, mask2, private_state_gch_->grid_width, private_state_gch_->grid_width);

            cv::Mat out;
            render_grid(g1, g2, cv::Size(10,10), out);
            return out;
        }
    }
    return cv::Mat();
}

void GridCompareHist::updateDynamicGui(QBoxLayout *layout)
{
    QVBoxLayout *internal_layout;
    if(container_hist_sliders_ != NULL) {
        container_hist_sliders_->deleteLater();
    }
    internal_layout = new QVBoxLayout;

    for(int i = 0 ; i < private_state_gch_->channel_count ; i++) {
        std::stringstream ch;
        ch << i + 1;

        int    default_bin = 32;
        double default_eps = 0.0;

        if(private_state_gch_->bins.size() < private_state_gch_->channel_count ) {
            private_state_gch_->bins.push_back(default_bin);
            private_state_gch_->eps.push_back(default_eps);
        } else {
            default_bin = private_state_gch_->bins[i];
            default_eps = private_state_gch_->eps[i];
        }

        QSlider *bins = QtHelper::makeSlider(internal_layout, "Ch." + ch.str() + " bins", default_bin, 1, 1000);
        QDoubleSlider *eps = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + " eps", default_eps, 0.0, 255.0, 0.01);
        addHistSliders(bins, eps);
    }

    container_hist_sliders_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_hist_sliders_);


}

void GridCompareHist::updateState(int value)
{
    if(!signalsBlocked()) {
        private_state_gch_->combo_index = combo_compare_->currentIndex();
        private_state_gch_->grid_width  = slide_width_->value();
        private_state_gch_->grid_height = slide_height_->value();
    }
}

void GridCompareHist::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);

    combo_compare_ = new QComboBox();
    combo_compare_->addItem("Correlation");
    combo_compare_->addItem("Chi-Square");
    combo_compare_->addItem("Intersection");
    combo_compare_->addItem("Hellinger");
    combo_compare_->addItem("Squared Distances");

    int index = combo_compare_->findText("Correlation");
    index_to_compare_.insert(intPair(index, CV_COMP_CORREL));
    index = combo_compare_->findText("Chi-Square");
    index_to_compare_.insert(intPair(index, CV_COMP_CHISQR));
    index = combo_compare_->findText("Intersection");
    index_to_compare_.insert(intPair(index, CV_COMP_INTERSECT));
    index = combo_compare_->findText("Hellinger");
    index_to_compare_.insert(intPair(index, CV_COMP_BHATTACHARYYA));
    index = combo_compare_->findText("Squared Distances");
    index_to_compare_.insert(intPair(index, AttrHistogram::CV_COMP_SQRD));

    layout->addWidget(combo_compare_);
    private_state_gch_->combo_index = combo_compare_->currentIndex();

    connect(combo_compare_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateState(int)));
    connect(slide_height_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
    connect(slide_width_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));

}

void GridCompareHist::prepareGrid(GridHist &g, const cv::Mat &img, const cv::Mat &mask, const int width, const int height)
{
    AttrHistogram::Params p;
    prepareHistParams(p.bins, p.ranges, p.eps);
    p.method = index_to_compare_[private_state_gch_->combo_index];
    cv_grid::prepare_grid<AttrHistogram>(g, img, height, width, p, mask, 1.0);
}

void GridCompareHist::addHistSliders(QSlider *bins, QDoubleSlider *eps)
{
    HistSliderPair p;
    p.first = bins;
    p.second = eps;
    hist_sliders_.push_back(p);
}

void GridCompareHist::prepareHistParams(cv::Mat &bins, cv::Mat &ranges, cv::Scalar &eps)
{
    bins = cv::Mat_<int>(private_state_gch_->channel_count, 1);
    ranges = cv::Mat_<float>(private_state_gch_->channel_count * 2 ,1);
    for(int i = 0 ; i < private_state_gch_->channel_count ; i++) {
        HistSliderPair p = hist_sliders_[i];
        bins.at<int>(i)     = p.first->value();
        ranges.at<float>(2 * i)     = 0.f;
        ranges.at<float>(2 * i + 1) = 256.f;
        eps[i]          = p.second->doubleValue();

        /// MEMENTO
        private_state_gch_->bins[i] = p.first->value();
        private_state_gch_->eps[i] = p.second->doubleValue();
    }
}

/// MEMENTO ------------------------------------------------------------------------------------
void GridCompareHist::setState(Memento::Ptr memento)
{
    state_.reset(new State);
    State::Ptr s = boost::dynamic_pointer_cast<State>(memento);
    assert(s.get());
    *boost::dynamic_pointer_cast<State>(state_) = *s;
    assert(state_.get());
    private_state_gch_ = boost::dynamic_pointer_cast<State>(state_).get();
    assert(private_state_gch_);

    blockSignals(true);
    slide_height_->setValue(private_state_gch_->grid_height);
    slide_width_->setValue(private_state_gch_->grid_width);
    combo_compare_->setCurrentIndex(private_state_gch_->combo_index);
    blockSignals(false);

    Q_EMIT modelChanged();
}

void GridCompareHist::State::readYaml(const YAML::Node &node)
{
    GridCompare::State::readYaml(node);
    node["compare"] >> combo_index;

    const YAML::Node &_bins = node["bins"];
    for(YAML::Iterator it = _bins.begin() ; it != _bins.end() ; it++) {
        int bin_val;
        *it >> bin_val;
        bins.push_back(bin_val);
    }

    const YAML::Node &_eps = node["eps"];
    for(YAML::Iterator it = _eps.begin() ; it != _eps.end() ; it++) {
        double eps_val;
        *it >> eps_val;
        eps.push_back(eps_val);
    }
}

GridCompareHist::State::State() :
    GridCompare::State(),
    combo_index(0)
{
}

Memento::Ptr GridCompareHist::getState() const
{
    State::Ptr memento(new State);
    *memento = *boost::dynamic_pointer_cast<State>(state_);
    return memento;
}

void GridCompareHist::State::writeYaml(YAML::Emitter &out) const
{
    GridCompare::State::writeYaml(out);

    out << YAML::Key << "compare" << YAML::Value << combo_index;
    out << YAML::Key << "bins" << YAML::Value << YAML::BeginSeq;
    for(std::vector<int>::const_iterator it = bins.begin() ; it != bins.end() ; it++) {
        out << *it;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "eps" << YAML::Value << YAML::BeginSeq;
    for(std::vector<double>::const_iterator it = eps.begin() ; it != eps.end() ; it++) {
        out << *it;
    }
    out << YAML::EndSeq;

}
