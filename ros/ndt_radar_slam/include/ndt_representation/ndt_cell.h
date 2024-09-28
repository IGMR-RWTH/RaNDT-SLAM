#ifndef NDT_CELL_H
#define NDT_CELL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include "ndt_slam/ndt_slam_parameters.h"

namespace rc {
namespace navigation {
namespace ndt {

class Cell {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief initialize new cell
    *  @param min_points_per_cell minimal number of points before calculating distribution
    *  @param params ndt cell parameters
    */
    void initialize(const int& min_points_per_cell, const NDTCellParameters& params);

    /** @brief add point to cell
     * @param point point to add
     * @param angle_dist angle and distance from sensor of added point for pNDT
     */
    void addPoint(const pcl::PointXYZI& point, const std::pair<double, double>& angle_dist);
   
    /** @brief add point cloud to cell
     * @param point point cloud to add
     * @param angle_dists angles and distances from sensor of added point for pNDT
     */
    bool addPointCloud(const pcl::PointCloud<pcl::PointXYZI>& point_cloud, const std::vector<std::pair<double, double>>& angle_dists);
    /** @brief get mean of cell
     */
    void getMean(Eigen::Vector2f& mean) const;

    /** @brief get mean of cell
     */
    inline const Eigen::Vector2f getMean() const {
      return new_mean_.block<2,1>(0,0);
    }

    /** @brief get mean of cell
     */
    void getIntensityMean(Eigen::Vector3f& mean) const;

    /** @brief get mean of cell
     */
    inline const Eigen::Vector3f getIntensityMean() const {
      return new_mean_;
    }

    /** @brief get mean intensity of cell
     */
    inline const double getMeanIntensity() const {
      return static_cast<double>(new_mean_(2));
    }
    
    /** @brief get max intensity of cell
     */
    inline const double getMaxIntensity() const {
      return max_intensity_;
    }
    
    /** @brief get covariance of cell
     */
    void getCov(Eigen::Matrix2f& cov) const;

    /** @brief get covariance of cell
     */
    inline const Eigen::Matrix2f getCov() const {
      return new_cov_.block<2,2>(0,0);
    }

    /** @brief get covariance of cell
     */
    void getIntensityCov(Eigen::Matrix3f& cov) const;

    /** @brief get covariance of cell
     */
    inline const Eigen::Matrix3f getIntensityCov() const {
      return new_cov_;
    }
    
    /** @brief evaluate normal distribution at point
     */
    double evaluateAtPoint(const Eigen::Vector2f& pt) const;

    /** @brief get mean and covariance of cell
     */
    void getMeanAndCov(Eigen::Vector2f& mean, Eigen::Matrix2f& cov) const;

    /** @brief get mean and covariance of cell
     */
    void getIntensityMeanAndCov(Eigen::Vector3f& mean, Eigen::Matrix3f& cov) const;
    
    /** @brief recalculates the values of the cell given the previously update cells
     */
    void updateCell(void);

    /** @brief clears values of the cell
     */
    void clearCell(void);

    /** @brief transform cell
     * @param trans transfromation to transform cell by
     */
    void transformCell(const Eigen::Affine2f& trans);

    /** @brief transform cell and underlying point cloud
     * @param trans transfromation to transform cell by
     */
    void transformCellWithPointCloud(const Eigen::Affine2f& trans);

    /** @brief get number of points in cell
     */
    size_t getNumCells() const;

    /** @brief l2 distance between two cells without intensity
     */
    double mahalanobisSquared(const Cell& subtrahend) const; 

    /** @brief l2 distance between two cells with intensity
     */
    double mahalanobisSquaredIntensity(const Cell& subtrahend) const; 
    
    /** @brief merge two cells
     */
    inline Cell& operator+=(const Cell& m_cell) {
      Eigen::Vector3f m_i_mean = m_cell.getIntensityMean();
      Eigen::Matrix3f m_i_cov = m_cell.getIntensityCov();
      size_t m_n_points = m_cell.getNumCells();
      this->new_cov_ = (this->n_points_-1) * this->new_cov_ + (m_n_points - 1 ) * m_i_cov + (((this->n_points_ * m_n_points)/(this->n_points_ + m_n_points))*((this->new_mean_-m_i_mean)*(this->new_mean_-m_i_mean).transpose()));
      this->new_mean_ = ((this->new_mean_ * this->n_points_) + (m_i_mean * m_n_points)) / (this->n_points_ + m_n_points);
      this->n_points_ += m_n_points;
      this->new_cov_ /= (this->n_points_-1);
      return *this;
    }

    /** @brief get pointCloud of cell
     */
    inline const pcl::PointCloud<pcl::PointXYZI>& getPointCloud() const {
      return points_;
    }

    /** @brief get angles and ranges of points generating the cell
     */
    inline const std::vector<std::pair<double, double>> getAngleDists() const {
      return polar_points_;
    }
    

  private:
    //void calculateCell();
    NDTCellParameters params_; /**parameters of cell*/
    pcl::PointCloud<pcl::PointXYZI> points_; /** points that contributed to the distribution */
    pcl::PointCloud<pcl::PointXYZI> points_to_add_; /** points that will be added next time updating the cell */
    std::vector<std::pair<double,double>> polar_points_; /** in pNDT angles and distances that contributed to the distribution*/
    std::vector<std::pair<double,double>> polar_points_to_add_; /** in pNDT angles and distances that will be added next time updating the cell*/
    unsigned int n_points_ = 0; /** number of contributing points */
    unsigned int n_points_to_add_ = 0; /** number of points that will be added */
    double mean_intensity_, max_intensity_; /** mean and max intensity */
    Eigen::Vector3f new_mean_; /** mean with intensity */
    Eigen::Matrix3f new_cov_; /** covariance with intensity */
    int min_points_per_cell_; /** minimal number of points to generate a distribution */
};
}
}
}
#endif

