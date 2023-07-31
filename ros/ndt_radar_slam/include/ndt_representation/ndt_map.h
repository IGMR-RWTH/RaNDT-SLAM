#ifndef NDT_MAP_H
#define NDT_MAP_H

#include <ndt_representation/ndt_cell.h>
#include "ndt_slam/ndt_slam_parameters.h"
#include <chrono>
#include <math.h>
#include <cfloat>

namespace rc {
namespace navigation {
namespace ndt {
  
class Map {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief initialize map
    * @param parameters parameters of the ndt map
    * @param center_x center in x coordinates should be 0
    * @param center_y center in y coordinates should be 0
    */ 
    void initialize(NDTMapParameters parameters, double center_x, double center_y);

    /** @brief insert point into map
    * @param point point to add
    * @param angle_dists angle and range for pNDT
    */ 
    void insertPoint(const pcl::PointXYZI& point, const std::pair<double,double>& angle_dist); 
    
    /** @brief insert point cluster into map
    * @param point points to add
    * @param angle_dists points in polar coordinates for pNDT
    */ 
    void insertCluster(const pcl::PointCloud<pcl::PointXYZI>& cluster, const std::vector<std::pair<double, double>>& angle_dists);

    /** @brief update map
    */
    void update(void);
    /** @brief clear map
    */
    void clear(void);
    /** @brief calculate CS divergence between maps
    */
    double calculateCSDivergence(const Map& m_map); 

    /** @brief get number of cells with distributions
    */
    inline unsigned int get_n_cells() const {
      return grid_.size();
      //return n_cells_;
    }

    /** @brief get cells at position i in vector
    */
    inline const Cell getCell(size_t i) {
      return grid_[i];
    }

    /** @brief get cell mean and covariance without intensity at position i in vector
    */
    bool getCellMeanAndCovariance(const unsigned int index, Eigen::Vector2f& mean, Eigen::Matrix2f& cov) const;

    /** @brief get cell mean and covariance with intensity at position i in vector
    */
    bool getCellMeanAndCovariance(const unsigned int index, Eigen::Vector3f& mean, Eigen::Matrix3f& cov) const;

    /** @brief get number of points in cell at position i in vector
    */
    inline const size_t getPointsInCell(size_t i) const {
      return grid_[i].getNumCells();
    }

    /** @brief transform map
    *  @param trans transformation to transform map by
    */
    void transformMap(const Eigen::Affine2f& trans);

    /** @brief transform map with pointss
    *  @param trans transformation to transform map by
    */
    void transformMapWithPointCloud(const Eigen::Affine2f& trans);

    /** @brief get index in grid given a coordinate
    *  @param point coordinate
    */
    inline unsigned int coordinateToIndex(const Eigen::Vector2f& point) const
    {
      return getIndex(static_cast<unsigned int>((point(0)-offset_x_)/res_), static_cast<unsigned int>((point(1)-offset_y_)/res_)); 
    }

    /** @brief merge map: moving cells are added to closest fixed cell
    */
    void mergeMapCell(const Map& moving_map);

    /** @brief merge map: transform point clouds and merge individual points 
    */
    void mergeMapPoints(const Map& moving_map);

    /** @brief get grid indizes of adjacent cells
    *  @param index grid index of point to search neighbors
    *  @param n_adjacent number of neighbors in each x- and y direction to search
    *  @param indizes resulting gric indizes
    */
    void getAdjacentIndizes(const unsigned int& index, const int& n_adjacent, std::vector<unsigned int>& indizes) const;
    
    /** @brief get closest cells of cell
    *  @param query_pt search neighbors close to this location
    *  @param n_neighbors number of neighbors
    *  @param indizes here, the resulting vector indizes are written in 
    */
    void getClosestCells(const Eigen::Vector2f& query_pt, const int& n_neighbours, std::vector<size_t>& indizes) const; 

    /** @brief get closest cells of cell
    *  @param query_pt search neighbors close to this cell
    *  @param n_neighbors number of neighbors
    *  @param indizes here, the resulting vector indizes are written in 
    */
    void getClosestCells(const Cell& query_cell, const int& n_neighbours, std::vector<size_t>& indizes) const;

    /** @brief evaluate ndt at point
    *  @param query_pt point at which to evaluate
    *  @param n_neighbors number of distributions to consider
    */
    double evaluateAtPoint(const Eigen::Vector2f& query_pt, const int& n_neighbours) const;
    
    /** @brief get vector of all ndt cells
    */
    inline std::vector<Cell> getCells() const {
      return grid_;
    }

    /** @brief insert new cell
    *  @param cell cell to insert 
    *  @return index of newly inserted cell
    */
    inline int insertCell(const Cell& cell) {
      grid_.push_back(cell);
      return grid_.size()-1;
    }

    /** @brief if true, no cells are in the grid
    */
    inline bool isEmpty() {
      return grid_.size() == 0;
    }

    /** @brief get vector indizes of all grid cellss
    *  @return the vector holding the vector indizes of the grid
    */
    inline std::vector<int> getGridIndizes() const {
      return grid_indizes_;
    }

  private:
    unsigned int size_x_, size_y_, n_cells_;  /** size of submap */
    double offset_x_, offset_y_;              /** offset of origin and corner */
    double res_;                              /** resolution */
    std::vector<int> grid_indizes_;           // for fast lookup
    NDTMapParameters parameters_;             /** ndt map parameters*/

    std::vector<Cell> grid_;                  /** this vector holds all generated ndt cells */

    Eigen::Affine2f robot_to_map_transform_ = Eigen::Affine2f::Identity(); /** transform between robot and global map frame (?)*/

    /** @brief get vector index corresponding to the cell of a point
    *  @param point query point
    *  @return vector index
    */
    inline unsigned int coordinateToIndex(const pcl::PointXYZI& point) const
    {
      return getIndex(static_cast<unsigned int>((point.x-offset_x_)/res_), static_cast<unsigned int>((point.y-offset_y_)/res_));
    }

    /**
     * @brief  Given two map coordinates... compute the associated index
     * @param mx The x coordinate
     * @param my The y coordinate
     * @return The associated index in the grid
     */
    inline unsigned int getIndex(const unsigned int& mx, const unsigned int& my) const
    {
      return my * size_x_ + mx;
    }

    /**
     * @brief  Given an index... compute the associated map coordinates
     * @param  index The index
     * @param  mx Will be set to the x coordinate
     * @param  my Will be set to the y coordinate
     */
    inline void indexToCells(unsigned int& index, unsigned int& mx, unsigned int& my) const
    {
      my = index / size_x_;
      mx = index - (my * size_x_);
    }

    size_t max_range_;
};

}
}    
} // namespace rc 

#endif