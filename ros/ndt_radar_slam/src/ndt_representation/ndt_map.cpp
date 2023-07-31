#include <ndt_representation/ndt_map.h>

namespace rc {
namespace navigation {
namespace ndt {

void Map::initialize(NDTMapParameters parameters, double center_x, double center_y) {
	parameters_ = parameters;
	size_x_ = parameters.size_x;
	size_y_ = parameters.size_y;
	n_cells_ = size_x_*size_y_;
	grid_.reserve(n_cells_);
	grid_indizes_.resize(n_cells_);
	for (size_t i = 0; i < n_cells_; i++) {
		grid_indizes_.at(i) = -1; // corresponds to empty
	}

	res_ = parameters.resolution;
	offset_x_ = -static_cast<double>(size_x_)/2.0*res_+ center_x;
	offset_y_ = -static_cast<double>(size_y_)/2.0*res_+ center_y;
}

bool Map::getCellMeanAndCovariance(const unsigned int index, Eigen::Vector2f& mean, Eigen::Matrix2f& cov) const {
	if (index < grid_.size()) {
		grid_.at(index).getMean(mean);
		grid_.at(index).getCov(cov);
		return true;
	}
	std::cout << "WARNING: requested cell out of range!" << "\n";
	return false;
}

bool Map::getCellMeanAndCovariance(const unsigned int index, Eigen::Vector3f& mean, Eigen::Matrix3f& cov) const {
	if (index < grid_.size()) {
		grid_.at(index).getIntensityMeanAndCov(mean, cov);
		return true;
	}
	std::cout << "WARNING: requested cell out of range!" << "\n";
	return false;
}

double Map::calculateCSDivergence(const Map& m_map) {
	double divergence;
	double interaction_term;
	double fixed_term;
	double moving_term;
	Eigen::Vector3f fixed_mean, query_mean;
	Eigen::Matrix3f fixed_cov, query_cov;
	std::vector<size_t> closest_neighbour_index;
	for (size_t f_grid_i = 0; f_grid_i < grid_.size(); f_grid_i ++) {
		
		// calculate interaction term
		closest_neighbour_index.clear();
		grid_.at(f_grid_i).getIntensityMeanAndCov(fixed_mean, fixed_cov);
		if (fixed_cov.determinant() < 0.00001) {
			continue;
		}
		for (size_t q_grid_i = 0; q_grid_i < m_map.get_n_cells(); q_grid_i++) {
			m_map.getCellMeanAndCovariance(q_grid_i, query_mean, query_cov);
			const Eigen::Vector3f mean_diff = fixed_mean - query_mean;
			const Eigen::Matrix3f mixed_cov = fixed_cov + query_cov;
			const Eigen::Matrix3f mixed_inf = mixed_cov.inverse();
			const double exp = mean_diff.transpose() * mixed_inf * mean_diff;
			interaction_term += (0.5/std::sqrt(M_PI * M_PI * mixed_cov.determinant())) * std::exp(-0.5 * exp);
		}

		//calculate pairs of fixed distributions
		if (fixed_cov.determinant() < 0.00001) {
			continue;
		}
		fixed_term += std::sqrt(fixed_cov.inverse().determinant())/(2*M_PI);
		for (size_t q_grid_i = 0; q_grid_i < f_grid_i; q_grid_i++) {
			grid_.at(q_grid_i).getIntensityMeanAndCov(query_mean, query_cov);
			const Eigen::Vector3f mean_diff = fixed_mean - query_mean;
			const Eigen::Matrix3f mixed_cov = fixed_cov + query_cov;
			const Eigen::Matrix3f mixed_inf = mixed_cov.inverse();
			const double exp = mean_diff.transpose() * mixed_inf * mean_diff;
			fixed_term += 2 * (0.5/std::sqrt(M_PI * M_PI * mixed_cov.determinant())) * std::exp(-0.5 * exp);
		}
	}
	for (size_t f_grid_i = 0; f_grid_i < m_map.get_n_cells(); f_grid_i++) {
		m_map.getCellMeanAndCovariance(f_grid_i, fixed_mean, fixed_cov);
		if (fixed_cov.determinant() < 0.00001) {
			continue;
		}
		moving_term += std::sqrt(fixed_cov.inverse().determinant())/(2*M_PI);
		for (size_t q_grid_i = 0; q_grid_i < f_grid_i; q_grid_i++) {
			m_map.getCellMeanAndCovariance(q_grid_i, query_mean, query_cov);
			const Eigen::Vector3f mean_diff = fixed_mean - query_mean;
			const Eigen::Matrix3f mixed_cov = fixed_cov + query_cov;
			const Eigen::Matrix3f mixed_inf = mixed_cov.inverse();
			const double exp = mean_diff.transpose() * mixed_inf * mean_diff;
			moving_term += 2 * (0.5/std::sqrt(M_PI * M_PI * mixed_cov.determinant())) * std::exp(-0.5 * exp);
		}
	}
	
	double cs_divergence = -std::log(interaction_term) + 0.5 * std::log(fixed_term) + 0.5 * std::log(moving_term);
	return cs_divergence;
}

void Map::getClosestCells(const Eigen::Vector2f& query_pt, const int& n_neighbours, std::vector<size_t>& indizes) const {
	std::vector<std::pair<double, size_t>> targets;       // pair of distance and index
	size_t center_index = coordinateToIndex(query_pt);    // index of center point
	int radius_increase = 0;                              // grid linf distance 

	std::vector<unsigned int> adjacent_indizes;
	while (targets.size() < n_neighbours && adjacent_indizes.size() < n_cells_) {  // not enough distributions found yet
		targets.clear();
		getAdjacentIndizes(center_index, radius_increase, adjacent_indizes);       // get all adjacent indices
		for (int i = 0; i < adjacent_indizes.size(); i++) {
			if (grid_indizes_.at(adjacent_indizes.at(i)) >= 0) {                   // only occupied cells
				double dist = (query_pt-grid_.at(grid_indizes_.at(adjacent_indizes.at(i))).getMean()).norm();  // grid-based distance
				targets.push_back(std::make_pair(dist, grid_indizes_.at(adjacent_indizes.at(i)))); // insert distance
			} 
		}
		radius_increase++;  // increase search radius
		if(radius_increase >= static_cast<int>(parameters_.max_neighbour_manhattan_distance/parameters_.resolution)) {
			break; // exceeds maximum linf distance
		}
	}
	
	std::sort(targets.begin(), targets.end());
	for (size_t i = 0; i < std::min(n_neighbours, (int) targets.size()); i++) {
		indizes.push_back(targets.at(i).second);  // return closest target
	}
}

void Map::getClosestCells(const Cell& query_cell, const int& n_neighbours, std::vector<size_t>& indizes) const {
	std::vector<std::pair<double, size_t>> targets;                  // pair of distance and index
	size_t center_index = coordinateToIndex(query_cell.getMean());   // index of center point
	int radius_increase = 0;                                         // grid linf distance 
	std::vector<unsigned int> adjacent_indizes; 
	while (targets.size() < n_neighbours && adjacent_indizes.size() < n_cells_) {  // not enough distributions found yet
		targets.clear();
		getAdjacentIndizes(center_index, radius_increase, adjacent_indizes);       // get all adjacent indices
		for (int i = 0; i < adjacent_indizes.size(); i++) {
			if (grid_indizes_.at(adjacent_indizes.at(i)) >= 0) {
				double dist = query_cell.mahalanobisSquaredIntensity(grid_.at(grid_indizes_.at(adjacent_indizes.at(i))));  // different metric: l2 between Distributions
				targets.push_back(std::make_pair(dist, grid_indizes_.at(adjacent_indizes.at(i))));
			}
		}
		radius_increase++;
		if(radius_increase >= static_cast<int>(parameters_.max_neighbour_manhattan_distance/parameters_.resolution)) {
			break;  // search radius exceeded
		}
	}
	std::sort(targets.begin(), targets.end());
	for (size_t i = 0; i < std::min(n_neighbours, (int) targets.size()); i++) {
		indizes.push_back(targets.at(i).second);  // return closest
	}
}

double Map::evaluateAtPoint(const Eigen::Vector2f& query_pt, const int& n_neighbours) const {  // evaluate NDT 
	std::vector<size_t> nn_indizes;
	getClosestCells(query_pt, n_neighbours, nn_indizes);  // evaluate only at closest cells for all points
	double result = 0;
	for (const size_t& idx: nn_indizes) {
		result += (1/n_neighbours) * grid_.at(idx).evaluateAtPoint(query_pt);
	}
	return result;
}

void Map::getAdjacentIndizes(const unsigned int& index, const int& n_adjacent, std::vector<unsigned int>& indizes) const {
	indizes.clear(); // return entire window of side length 2*n_adjacent+1
	for (int n = 0; n<=n_adjacent; n++) {
		for (int i = -n_adjacent; i <= n_adjacent; i++){
			for (int j = -n_adjacent; j <= n_adjacent; j++){
				unsigned int new_index = static_cast<unsigned int>(index + i + j*size_x_);
            	if (new_index < n_cells_ && std::find(indizes.begin(), indizes.end(), new_index) == indizes.end()) {
					indizes.push_back(new_index);
				}
			}
		} 
	}
}

void Map::transformMap(const Eigen::Affine2f& trans) {
  	for (Cell &cell : grid_) {
		cell.transformCell(trans);
	}
	robot_to_map_transform_ = robot_to_map_transform_ * trans;
}

void Map::transformMapWithPointCloud(const Eigen::Affine2f& trans) {
  	for (Cell &cell : grid_) {
		cell.transformCellWithPointCloud(trans);
	}
	robot_to_map_transform_ = robot_to_map_transform_ * trans;
}

void Map::mergeMapCell(const Map& moving_map) {
	std::vector<Cell> cells = moving_map.getCells();
	for (size_t i = 0; i < moving_map.get_n_cells(); i++) {
		Cell m_cell = cells.at(i);
		size_t map_index = coordinateToIndex(m_cell.getMean());
		if (map_index < grid_indizes_.size()) {
			int index = grid_indizes_.at(map_index);
			if (index >=0) {
				grid_.at(index) += m_cell;  // if cell already holds distribution, use recursive update equation
			} else {
				int idx = insertCell(m_cell);
				grid_indizes_.at(map_index) = idx;  // else, simply insert new cell
			}
		}
		
	}
}

void Map::mergeMapPoints(const Map& moving_map) {
	std::vector<Cell> cells = moving_map.getCells();
	for (size_t i = 0; i < moving_map.get_n_cells(); i++) {
		Cell m_cell = cells.at(i);
		pcl::PointCloud<pcl::PointXYZI> points = m_cell.getPointCloud();
		std::vector<std::pair<double,double>> angle_dists = m_cell.getAngleDists();
		for (size_t j = 0; j < m_cell.getNumCells(); j++) {
			insertPoint(points.at(j), angle_dists.at(j)); // insert each point of point cloud individually
		}
	}
	update();  // update all cell parameters
}

void Map::insertPoint(const pcl::PointXYZI& point, const std::pair<double,double>& angle_dist) {
	unsigned int index = coordinateToIndex(point);  // get corresponding index
	if (index < n_cells_) {
		if (grid_indizes_.at(index) >= 0) { // if distribution exists
			grid_.at(grid_indizes_.at(index)).addPoint(point, angle_dist); //add point
		} 
		else { // else create new cell
			Cell cell;
			cell.initialize(parameters_.min_points_per_cell, parameters_.ndt_cell_parameters);
			cell.addPoint(point, angle_dist);
			int grid_idx = insertCell(cell);
			grid_indizes_.at(index) = grid_idx;
		}
	}
}

void Map::insertCluster(const pcl::PointCloud<pcl::PointXYZI>& cluster, const std::vector<std::pair<double, double>>& angle_dists) {
	Cell cell;
	cell.initialize(parameters_.min_points_per_cell, parameters_.ndt_cell_parameters); // new cell generated by cluster
	if (cell.addPointCloud(cluster, angle_dists)) {
		grid_indizes_.at(coordinateToIndex(cell.getMean())) = grid_.size(); // update index with new position in vector
		grid_.push_back(cell);
	}
}

void Map::update(void) {
    for (size_t i=0; i<grid_.size(); i++) {
		grid_.at(i).updateCell(); // update all cells
	}
}
void Map::clear(void) {
	
	for (unsigned int i = 0; i < grid_.size(); i++)  {
		grid_.at(i).clearCell();  // clear cells
	}
	grid_.clear();
	
}

}
}
}