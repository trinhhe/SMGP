#ifndef _SPATIAL_INDEX_HPP__
#define _SPATIAL_INDEX_HPP__

#include <vector>
#include <Eigen/Dense>

class SpatialIndex {
public:
	/// Spatial Index grid
	std::vector<std::vector<std::vector<std::vector<int>>>> grid;
	/// Original mesh
	Eigen::MatrixXd mesh;											

	/// Populate spatial index, Input: mesh V
	void populate(Eigen::MatrixXd & V);
	/// Clear spatial index by deleting all it contains
	void clear();
	/// Compute closest point to a given point, Input: point, Outputs: closest point and index of closest point
	void closest(Eigen::Vector3d & point, Eigen::Vector3d & result, int & result_index);
	/// Compute grid coordinates from point, Input: point, Output: tuple of coordinates (x,y,z)
	std::tuple <int, int, int> compute_index(Eigen::Vector3d point);

private:
	Eigen::Vector3d  bb_min, bb_max, dim;
	int grid_resolution_x, grid_resolution_y, grid_resolution_z;
	double gridsize;
};

#endif // _SPATIAL_INDEX_HPP__
