#ifndef _UTILS_HPP__
#define _UTILS_HPP__

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <string>

using namespace std;
using namespace Eigen;

/**
 * Read a landmarks file and store the landmarks in ml_v sorted by landmark_id in ascending order
 * @param fileneme: full path to the landmark file
 * @param lm_v: eigen vector that will hold the landmarks
 */
void read_landmarks(const string &filename, VectorXi &lm_v);

/**
 * Given a vector or landmarks and its corresponding ids write those to a landmark file named filename
 * @param filename: full path of the file that will be written.
 * The file format will be <landmark_vi landmark_vi_id> for each line of the file
 * @param lm_v: eigen vector that holds the landmarks
 * @param lm_ids: eigen vector that holds the landmark_ids
 */
void write_landmarks(const string &filename, VectorXi &lm_v, VectorXi &lm_ids);

/**
 * Read a face mesh from a file and load its associated landmarks if present.
 */
void select_face(std::string &face_file);

void concat_meshes(MatrixXd &V1, MatrixXi &F1, MatrixXd &V2, MatrixXi &F2, MatrixXd &V, MatrixXi &F);

void show_landmarks(Eigen::VectorXi landmarks);

void landmarks_positions(Eigen::MatrixXd &V, Eigen::VectorXi &landmarks, Eigen::MatrixXd &positions);

#endif // _UTILS_HPP__
