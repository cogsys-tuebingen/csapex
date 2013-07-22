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
class GridT {
public:
    /**
     * @brief Return amount of rows.
     * @return          the amount of rows
     */
    int rows() const
    {
        return rows_;
    }

    /**
     * @brief Return amount of cols.
     * @return          the amount of cols
     */
    int cols() const
    {
        return cols_;
    }

protected:
    GridT(const int rows, const int cols) :
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
class Grid_ : public GridT
{
public:
    /**
     * @brief Grid default constructor.
     */
    Grid_() :
        GridT(0,0)
    {
    }

    /**
     * @brief Grid constructor.
     * @param rows      the amount of rows
     * @param cols      the amount of cols
     */
    Grid_(const int rows, const int cols) :
        GridT(rows, cols),
        cells_(new Cell[rows * cols])
    {
    }

    /**
     * @brief Grid copy constructor.
     * @param g         another grid
     */
    Grid_(const Grid_& g, const bool deep_copy = false) :
        GridT(g.rows_, g.cols_)
    {
        if(deep_copy) {
            cells_.reset(new Cell[g.rows_ * g.cols_]);
            std::copy(g.cells_.get(), g.cells_.get() + cols_ * rows_, cells_.get());
        } else {
            cells_.reset(g.cells_.get());
        }
    }

    Grid_(const Grid_& g, const Rect &roi, const bool deep_copy = false) :
        GridT(g.rows_, g.cols_)
    {

        if(deep_copy) {
            cells_.reset(new Cell[roi.width * roi.height]);
            int limit  = max_row_ * step_+ max_col_;
            int offset = min_row_ * step_+ min_col_;
            std::copy(g.cells_.get() + offset, g.cells_.get() + limit, cells_.get());
        } else {
            cells_.reset(g.cells_.get());
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
        setROI_(roi.y, roi.x, roi.height, roi.width);
    }

    void setROI(int row, int col, int height, int width)
    {
        assert(row < rows_);
        assert(col < cols_);
        assert(row + height <= rows_);
        assert(col + width <= cols_);

        min_row_ = row;
        min_col_ = min_col_;
        max_row_ = row + height;
        max_col_ = col + width;
    }

    void resetROI()
    {
        min_row_= 0;
        min_col_= 0;
        max_row_= rows_;
        max_col_= cols_;
    }

    Grid_& getROI(Rect roi, const bool deep_copy = false)
    {
        return Grid_(&this, roi, deep_copy);
    }

    Grid_& getROI(const int x, const int y, const int width, const int height, const bool deep_copy = false)
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
    bool operator == (const Grid_<Cell, Rect> &g)
    {
        assert(max_col_ - min_col_ == g.cols_);
        assert(max_row_ - min_row_ == g.rows_);

        bool res = true;

        for(int i = 0 ; i < g.rows_ ; i++) {
            for(int j = 0 ; j < g.cols_ ; g++) {
                res &= g.cells_[(i + g.min_row_) * g.step_ + j + g.min_col_] == cells_[(i + min_row_) * step_ + min_col_ + j];
            }
        }
        return res;
    }

private:
    boost::shared_array<Cell> cells_;
};
}
#endif // GRID_HPP
