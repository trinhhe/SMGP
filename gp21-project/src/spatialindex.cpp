#include "spatialindex.hpp"

#include <iostream>
using namespace std;


void SpatialIndex::populate(Eigen::MatrixXd & V) {
    mesh = V;
    bb_min = V.colwise().minCoeff();
    bb_max = V.colwise().maxCoeff();
    dim = (bb_max - bb_min);

    // Resize grid
    gridsize = 0.1 * dim.norm();
    grid_resolution_x = ceil(dim[0] / gridsize);
    grid_resolution_y = ceil(dim[1] / gridsize);
    grid_resolution_z = ceil(dim[2] / gridsize);

    grid.resize(grid_resolution_x);
    for (int i = 0; i < grid_resolution_x; i++)
    {
        grid[i].resize(grid_resolution_y);
        for (int j = 0; j < grid_resolution_y; j++)
        {
            grid[i][j].resize(grid_resolution_z);
        }
    }

    // Populate grid
    int x, y, z;
    for (int i = 0; i < V.rows(); i++) {
        tie(x, y, z) = compute_index(V.row(i));
        grid[x][y][z].push_back(i);
    }
}

void SpatialIndex::clear() {
    // Clear grid
    grid.clear();
}

void SpatialIndex::closest(Eigen::Vector3d & point, Eigen::Vector3d & result, int & result_index) {
    int x, y, z;
    tie(x, y, z) = compute_index(point);
    result_index = -1;
    double dist, min_dist;
    Eigen::Vector3d temp;

    // search for closest point
    int numberOfCells = 0;
    while (result_index == -1) {
        for (int i = -numberOfCells; i <= numberOfCells; i++) {
            if (x + i >= grid_resolution_x || x + i < 0) { continue; }
            for (int j = -numberOfCells; j <= numberOfCells; j++) {
                if (y + j >= grid_resolution_y || y + j < 0) { continue; }
                for (int k = -numberOfCells; k <= numberOfCells; k++) {
                    if (z + k >= grid_resolution_z || z + k < 0) { continue; }
                    for (int l = 0; l < grid[x + i][y + j][z + k].size(); l++) {
                        temp = mesh.row(grid[x + i][y + j][z + k][l]);
                        dist = (temp - point).norm();
                        if (result_index == -1) {
                            min_dist = dist;
                            result_index = grid[x + i][y + j][z + k][l];
                        }
                        if (dist < min_dist) {
                            result_index = grid[x + i][y + j][z + k][l];
                            min_dist = dist;
                        }
                    }
                }
            }
        }
        numberOfCells++;
    }
    result = mesh.row(result_index);
}

std::tuple <int, int, int> SpatialIndex::compute_index(Eigen::Vector3d point) {
    // Spatial index of given point
    Eigen::Vector3d coordinates = (point - bb_min) / gridsize;
    int x = max(floor(coordinates[0]), 0.);
    int y = max(floor(coordinates[1]), 0.);
    int z = max(floor(coordinates[2]), 0.);
    return  { min(x, grid_resolution_x), min(y, grid_resolution_y), min(z, grid_resolution_z) };
}