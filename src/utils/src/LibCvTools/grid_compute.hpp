#ifndef CV_GRID_HPP
#define CV_GRID_HPP
#include "grid.hpp"
#include "grid_attributes.hpp"

/**
 * @namespace Common operations for opencv when a grid is needed.
 */
namespace cv_grid {
/**
 * @brief Prepare a homogene grid using hls attributes for cell comparison.
 * @param grid      the reference the grid will be written to
 * @param img       the source image
 * @param rows      the amount of rows
 * @param cols      the amount of cols
 * @param eps       the mean error
 */
template<class Attribute, class GridT, class Params>
inline void  prepare_grid(GridT &grid, const cv::Mat &img, const int rows, const int cols, const Params &p, const cv::Mat &mask = cv::Mat(), const double thresh = 1.0)
{
    grid = GridT(rows,cols);

    int cell_height      = img.rows / rows;
    int cell_width       = img.cols / cols;
    int cell_height_rest = img.rows % rows;
    int cell_width_rest  = img.cols % cols;

    for(int i = 0 ; i < cols ; i++) {
        for(int j = 0 ; j < rows ; j++) {
            cv::Rect r(cell_width * i, cell_height * j, cell_width, cell_height);

            if(i == cols - 1)
                r.width  += cell_width_rest;
            if(j == rows - 1)
                r.height += cell_height_rest;


            if(!mask.empty()) {
                cv::Mat mask_roi(mask, r);
                cv::Scalar s = cv::sum(mask_roi);
                if(s[0] / (double)(r.width * r.height) < thresh) {
                    grid(i,j) = GridCell<cv::Rect, Attribute>(false);
                    continue;
                }
            }

            cv::Mat img_roi(img, r);
            Attribute attr = Attribute::generate(img_roi, p);
            grid(j,i) = GridCell<cv::Rect, Attribute>(r, attr);
        }
    }
}

typedef GridCell<cv::Rect, AttrScalar>          GridCellScalar;    /// a grid cell using hls attributes
typedef Grid_<GridCellScalar, cv::Rect>         GridScalar;        /// a grid based on hls grid cells
typedef GridCell<cv::Rect, AttrHistogram>       GridCellHist;
typedef Grid_<GridCellHist, cv::Rect>           GridHist;

template<class GridT>
inline void render_grid_count(const GridT &g1, const GridT &g2, const cv::Size &block_size, cv::Mat &out, std::pair<int, int> &counts, int &valid)
{
    out = cv::Mat(block_size.height * g1.rows() + 40, block_size.width * g1.cols(), CV_8UC3, cv::Scalar(0,0,0));
    counts.first = 0;
    counts.second = 0;
    valid = 0;
    for(int i = 0 ; i < g1.rows() ; i++) {
        for(int j = 0 ; j < g1.cols() ; j++) {

            cv::Rect r = cv::Rect(j * block_size.width, i * block_size.height , block_size.width, block_size.height);
            if(!g1(i,j).enabled || !g2(i,j).enabled) {
                cv::rectangle(out, r, cv::Scalar(0, 255, 255), CV_FILLED);
                continue;
            }

            bool cell_compare = g1(i,j) == g2(i,j);
            if(cell_compare) {
                cv::rectangle(out, r, cv::Scalar(0, 255, 0), CV_FILLED);
                counts.first++;
            } else {
                cv::rectangle(out, r, cv::Scalar(0, 0, 255), CV_FILLED);
                counts.second++;
            }
            valid++;
        }
    }

    if(out.rows < 44 && out.cols < 120)
        throw std::runtime_error("Image to small to render text!");
    std::stringstream text;
    text << "+" << counts.first << " | -" << counts.second << " | all: " << valid;
    cv::putText(out, text.str(), cv::Point(0, out.rows - 4), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 255), 2);

}

template<class GridT>
inline void render_grid(const GridT &g1, const GridT &g2, const cv::Size &block_size, cv::Mat &out)
{
    std::pair<int,int> c(0,0);
    int v = 0;
    render_grid_count<GridT>(g1, g2, block_size, out, c, v);
}

}

template<class GridT>
inline void grid_count(const GridT &g1, const GridT &g2, std::pair<int, int> &counts, int &valid)
{
    counts.first = 0;
    counts.second = 0;
    valid = 0;
    for(int i = 0 ; i < g1.cols() ; i++) {
        for(int j = 0 ; j < g1.rows() ; j++) {
            bool cell_compare = g1(j,i) == g2(j,i);
            if(cell_compare) {
                counts.first++;
            } else {
                counts.second++;
            }
            valid++;
        }
    }
}

template<class GridT>
inline void grid_heatmap(GridT &g1, GridT &g2, cv::Mat &vals)
{
    assert(g1.cols() <= g2.cols());
    assert(g1.rows() <= g2.rows());

    int col_iterations = g2.cols() - g1.cols() + 1;
    int row_iterations = g2.rows() - g1.rows() + 1;

    vals = cv::Mat(row_iterations, col_iterations, CV_32F, cv::Scalar::all(0));
    for(int i = 0 ; i <  col_iterations ; i++) {
        for(int j = 0 ; j < row_iterations ; j++) {
            std::pair<int, int> counts;
            int valid;
            g2.setROI(j,i, g1.rows(), g1.cols());
            grid_count(g1, g2, counts, valid);
            g2.resetROI();
            if(valid > 0) {
                vals.at<float>(row_iterations, col_iterations) = counts.first / (float) valid;
            }
        }
    }
}

inline void render_heatmap(const cv::Mat &values, const cv::Size &block_size, cv::Mat &out)
{
    out = cv::Mat(values.rows * block_size.height, values.cols * block_size.width, CV_8UC1);
    for(int i = 0 ; i < values.rows ; i++) {
        for(int j = 0 ; j < values.cols ; j++) {
            int value = std::floor(values.at<float>(i,j) * 255 + 0.5);
            cv::Rect r  = cv::Rect(j * block_size.width,i * block_size.height,block_size.width,block_size.height);
            cv::rectangle(out,r,cv::Scalar(value), CV_FILLED);
        }
    }
}


#endif // CV_GRID_HPP
