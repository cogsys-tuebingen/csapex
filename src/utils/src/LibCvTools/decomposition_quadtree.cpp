#include "decomposition_quadtree.h"

QuadtreeDecomposition::QuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size, DecompositionClassifier &_classifier) :
    image_(_image),
    min_region_size_(_min_region_size),
    classfier_(_classifier),
    quadtree_root_(cv::Rect(0,0,_image.cols, _image.rows)),
    auto_iterate_(false),
    debug_size_(-1,-1)
{
    classfier_.set_image(_image);
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
         if(classfier_.classify(quadtree_root_))
            split_and_activate(quadtree_root_);
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

void QuadtreeDecomposition::process_active_nodes()
{
    CVQtNodesList list = quadtree_nodes_;
    quadtree_nodes_.clear();

    for(CVQtNodesList::iterator it = list.begin() ; it != list.end() ; it++){
        CVQt *node = *it;
        if(!min_size_reached(*node) && classfier_.classify(*node)) {
            split_and_activate(*node);
        }
    }
}

void QuadtreeDecomposition::render_debug()
{
    cv::Mat img_out = image_.clone();

    CVQtNodesList leaves;
    quadtree_root_.collect_leaves(leaves);
    for(CVQtNodesList::iterator it = leaves.begin() ; it != leaves.end() ; it++) {
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
