#include "decomposition_quadtree.h"

QuadtreeDecomposition::QuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size, DecompositionClassifier::Ptr _classifier) :
    image_(_image),
    min_region_size_(_min_region_size),
    classifier_(_classifier),
    auto_iterate_(false),
    debug_size_(-1,-1)
{
    int divisor = gcd(_image.cols, _image.rows);
    int x = _image.cols / divisor;
    int y = _image.rows / divisor;
    cv::Rect region;
    region.width = _image.cols / x;
    region.height = _image.rows / y;

    for(int i = 0 ; i < x ; ++i) {
        for(int j = 0 ; j < y ; ++j) {
            region.x = i * region.width;
            region.y = j * region.height;
            quadtree_roots_.push_back(CVQt(region));
        }
    }
    classifier_->set_image(_image);
}

QuadtreeDecomposition::~QuadtreeDecomposition()
{
}

void QuadtreeDecomposition::auto_iterate()
{
    while(iterate());
}

bool QuadtreeDecomposition::iterate()
{
    if(quadtree_nodes_.size() == 0) {
        for(CVQtNodesList::iterator it = quadtree_roots_.begin() ; it != quadtree_roots_.end() ; ++it) {
            if(classifier_->classify(*it) && !min_size_reached(*it))
                split_and_activate(*it);
            else
                quadtree_leaves_.push_back(&(*it));
        }
    } else {
        process_active_nodes();
    }

    if(debug_size_.width != -1 && debug_size_.height != -1)
        render_debug();

    return quadtree_nodes_.size() != 0;
}


void QuadtreeDecomposition::enable_debug_out_put(const cv::Size &window_size, const cv::Scalar &debug_color)
{
    debug_size_ = window_size;
    debug_color_ = debug_color;
}

void QuadtreeDecomposition::regions(std::vector<cv::Rect> &regions)
{
    for(CVQtNodesListPtr::iterator it = quadtree_leaves_.begin() ; it != quadtree_leaves_.end() ; ++it) {
        CVQt *node = *it;
        cv::Rect rect = *node;
        limit(rect, image_.cols, image_.rows);
        regions.push_back(rect);
    }
}

void QuadtreeDecomposition::process_active_nodes()
{
    CVQtNodesListPtr list = quadtree_nodes_;
    quadtree_nodes_.clear();

    for(CVQtNodesListPtr::iterator it = list.begin() ; it != list.end() ; ++it){
        CVQt *node = *it;
        if(!min_size_reached(*node) && classifier_->classify(*node)) {
            split_and_activate(*node);
        } else {
            quadtree_leaves_.push_back(node);
        }
    }
}

void QuadtreeDecomposition::render_debug()
{
    cv::Mat img_out = image_.clone();

    for(CVQtNodesListPtr::iterator it = quadtree_leaves_.begin() ; it != quadtree_leaves_.end() ; ++it) {
        CVQt *node = *it;
        cv::Rect rect = *node;
        limit(rect, img_out.cols, img_out.rows);

        cv::rectangle(img_out,rect, debug_color_);
    }
    std::stringstream s;
    s << "/tmp/" << quadtree_nodes_.size() << ".jpg";
    cv::imwrite(s.str(), img_out);
    if(debug_size_.width > 0 && debug_size_.height > 0)
        cv::resize(img_out, img_out, debug_size_);

    cv::imshow("QuadtreeDecom", img_out);
    cv::waitKey(0);
}
