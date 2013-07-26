#ifndef GRID_HPP
#define GRID_HPP
#include <vector>
#include <assert.h>
#include <boost/shared_array.hpp>

/**
 * @class The GridCell class is used as the content for a cell of a grid.
 *        An attribute is a value that can be used to compare two cells.
 */
namespace cv_grid {
/**
 * @brief The Grid container.
 */
class Grid {
public:
    /**
     * @brief Return amount of rows.
     * @return          the amount of rows
     */
    int rows() const
    {
        return max_row_ - min_row_;
    }

    /**
     * @brief Return amount of cols.
     * @return          the amount of cols
     */
    int cols() const
    {
        return max_col_ - min_col_;
    }

    virtual void resetROI()
    {
        min_row_= 0;
        min_col_= 0;
        max_row_= rows_;
        max_col_= cols_;
    }

    virtual void setROI(int row, int col, int height, int width)
    {
        assert(min_row_ + row < rows_);
        assert(min_col_ + col < cols_);
        assert(min_row_ + row + height <= rows_);
        assert(min_col_ + col + width <= cols_);

        min_row_ += row;
        min_col_ += col;
        max_row_ = min_row_ + height;
        max_col_ = min_col_ + width ;
    }



protected:
    Grid(const int rows, const int cols) :
        rows_(rows),
        cols_(cols),
        min_row_(0),
        min_col_(0),
        max_row_(rows),
        max_col_(cols),
        step_(cols)
    {
    }

    int rows_;          /// amount of rows
    int cols_;          /// amount of cols
    int min_row_;
    int min_col_;
    int max_row_;
    int max_col_;
    int step_;
};
/**
 * @brief A grid cell.
 */
template<class Rect, class Attributes>
class GridCell {
public:
    /**
     * @brief GridCell default constructor.
     */
    GridCell(const bool _enabled = true) :
        enabled(_enabled)
    {
    }

    /**
     * @brief GridCell constructor.
     * @param _bounding         the grid cells bounding box
     * @param _attributes       the grid cells attributes
     */
    GridCell(const Rect _bounding, const Attributes _attributes) :
        bounding(_bounding),
        attributes(_attributes),
        enabled(true)
    {
    }

    /**
     * @brief GridCell copy constructor.
     * @param c     another grid cell
     */
    GridCell(const GridCell<Rect, Attributes> &c) :
        bounding(c.bounding),
        attributes(c.attributes),
        enabled(true)
    {
    }

    /**
     * @brief ~GridCell default destructor.
     */
    virtual ~GridCell()
    {
    }

    /**
     *  Calculate the grid cells center and retrun it.
     *  @param p    the reference the result will be written to
     */
    template<class Point>
    void get_center(Point &p)
    {
        p.x = bounding.x + bounding.width / 2.0;
        p.y = bounding.y + bounding.height / 2.0;
    }

    /**
     * @brief Compare two grid cells
     * @param g     another grid cell
     * @return      if the cells are equal in a defined way
     */
    bool operator == (const GridCell<Rect, Attributes> g) const
    {
        return g.attributes == attributes;
    }

    /**
     * @brief Check if cell is enabled.
     * @return
     */
    bool is_enbabled()
    {
        return enabled;
    }

    Rect        bounding;           /// bounding box of the grid
    Attributes  attributes;         /// the attributes to compare
    bool        enabled;
};

/**
 * @class   The Grid class can be used to make the usage of a Grid easier.
 *          It can be observed as a functional wrapper class.
 */
template<class Cell, class Rect>
class Grid_ : public Grid
{
public:
    /**
     * @brief Grid default constructor.
     */
    Grid_() :
        Grid(0,0)
    {
    }

    /**
     * @brief Grid constructor.
     * @param rows      the amount of rows
     * @param cols      the amount of cols
     */
    Grid_(const int rows, const int cols) :
        Grid(rows, cols),
        cells_(new Cell[rows * cols])
    {
    }

    /**
     * @brief Grid copy constructor.
     * @param g         another grid
     */
    Grid_(const Grid_& g, const bool deep_copy = false) :
        Grid(g.rows(), g.cols())
    {
        if(deep_copy) {
            cells_.reset(new Cell[rows_ * cols_]);
            copy(g);
        } else {
            cells_ = boost::shared_array<Cell>(g.cells_);
        }
    }

    Grid_(const Grid_& g, const Rect &roi, const bool deep_copy = false) :
        Grid(g.rows_, g.cols_)
    {

        if(deep_copy) {
            cells_.reset(new Cell[roi.width * roi.height]);
            setROI(roi);
            copy(g);
        } else {
            cells_ = boost::shared_array<Cell>(g.cells_);
            setROI(roi);
        }
    }

    /**
     * @brief ~Grid default destructor.
     */
    virtual ~Grid_()
    {
    }

    void setROI(Rect roi)
    {
        Grid::setROI(roi.y, roi.x, roi.height, roi.width);
    }

    void setROI(int row, int col, int height, int width)
    {
        Grid::setROI(row, col, height, width);
    }

    void resetROI()
    {
        Grid::resetROI();
    }

    Grid_ getROI(const Rect roi, const bool deep_copy = false)
    {
        return Grid_(*this, roi, deep_copy);
    }

    Grid_ getROI(const int y, const int x, const int height, const int width, const bool deep_copy = false)
    {
        return getROI(Rect(x,y,width,height), deep_copy);
    }

    Grid_& operator = (const Grid_ &g)
    {
        cols_    = g.cols_;
        rows_    = g.rows_;
        min_row_ = g.min_row_;
        min_col_ = g.min_col_;
        max_row_ = g.max_row_;
        max_col_ = g.max_col_;
        step_    = g.step_;

        int elements = g.cols_ * g.rows_;
        cells_.reset(new Cell[elements]);
        std::copy(g.cells_.get(), g.cells_.get() + elements, cells_.get());
        return *this;
    }

    /**
     * @brief Access a cell.
     * @param row       row index
     * @param cols      col index
     * @return          a reference to the cells
     */
    Cell & operator() (const unsigned int row, const unsigned int col)
    {
        assert(min_col_ + col < max_col_);
        assert(min_row_ + row < max_row_);

        return cells_[min_col_ + col + (min_row_ + row) * step_];
    }

    /**
     * @brief Access a cell.
     * @param row       row index
     * @param cols      col index
     * @return          a const reference
     */
    Cell & operator() (const unsigned int row, const unsigned int col) const
    {
        assert(min_col_ + col < max_col_);
        assert(min_row_ + row < max_row_);

        return const_cast<Cell&> (cells_[min_col_ + col + (min_row_ + row) * step_]);
    }

    /**
     * @brief Compare to grids.
     * @param g         another grid
     * @return          if the grids are equal
     */
    bool operator == (const Grid_<Cell, Rect> &g) const
    {
        assert(cols() == g.cols());
        assert(cols() == g.cols());

        bool res = true;

        for(int i = 0 ; i < g.rows_ ; i++) {
            for(int j = 0 ; j < g.cols_ ; j++) {
                res &= g.cells_[(i + g.min_row_) * g.step_ + j + g.min_col_] == cells_[(i + min_row_) * step_ + min_col_ + j];
            }
        }
        return res;
    }

    void compare(const Grid_ &g, std::pair<int, int> &counts, int &valid) const
    {
        assert(max_col_ - min_col_ == g.cols());
        assert(max_row_ - min_row_ == g.rows());

        counts.first = 0;
        counts.second = 0;
        valid = 0;

        for(int i = 0 ; i < g.rows() ; i++) {
            for(int j = 0 ; j < g.cols() ; j++) {
                int pos_g = (i + g.min_row_) * g.step_ + j + g.min_col_;
                int pos   = (i + min_row_) * step_ + min_col_ + j;

                if(!(g.cells_[pos_g].enabled && cells_[pos].enabled))
                    continue;

                bool cell_compare = g.cells_[pos_g] == cells_[pos];

                if(cell_compare) {
                    counts.first++;
                } else {
                    counts.second++;
                }
                valid++;
            }
        }
    }

//    void parallel_compare(Grid_ &g, std::pair<int, int> &counts, int &valid)
//    {
//        assert(max_col_ - min_col_ == g.cols());
//        assert(max_row_ - min_row_ == g.rows());

//        counts.first = 0;
//        counts.second = 0;
//        valid = 0;

//        for(int i = 0 ; i < g.rows() ; i++) {
//            Grid_ roi2 = g.getROI(0, i, g.cols(), 1);
//            Grid_ roi1 = getROI(0, i, cols(), 1);
//            std::pair<int,int> tmp_counts;
//            int                tmp_valid;
//            roi2.compare(roi1, tmp_counts, tmp_valid);
//            counts.first += tmp_counts.first;
//            counts.second += tmp_counts.second;
//            valid += tmp_valid;
//        }
//        boost::threadpool::thread_pool p(3);
//        p.wait(0);
// Grid_& getROI(const int x, const int y, const int width, const int height, const bool deep_copy = false)

//    }

private:
    boost::shared_array<Cell> cells_;

    void copy(const Grid_& src)
    {
        for(int i = 0 ; i < rows_ ; i++) {
            for(int j = 0 ; j < cols_ ; j++) {
                cells_[i * step_ + j]  = src.cells_[(i + src.min_row_) * src.step_ + j + src.min_col_];
            }
        }
    }

};
}
#endif // GRID_HPP
