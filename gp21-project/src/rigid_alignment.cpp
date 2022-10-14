#include <igl/slice.h>

#include "rigid_alignment.hpp"
#include "utils.hpp"
#include "globals.hpp"

using namespace Eigen;
using namespace std;

/// computes the scale of the given landmarks, as their average distance to their mean.
double landmark_scale(const VectorXi &landmarks, const MatrixXd &V) {
	auto landmark_positions = igl::slice(V, landmarks, 1);
	RowVector3d mean_landmark = landmark_positions.colwise().mean();
	MatrixXd distances = landmark_positions.rowwise() - mean_landmark;
	return distances.rowwise().norm().mean();
}

bool rigidly_align_to_template(const VectorXi &landmarks, MatrixXd &V) {
	VectorXi &template_landmarks = template_landmark_vertices;
	if (template_landmarks.size() != landmarks.size()) {
		cerr << "landmark sets to align have incompatible count: "
			<< landmarks.size() << " on current face ≠ "
			<< template_landmarks.size() << " on template."
			<< endl;
		return false;
	}

	MatrixXd &template_vertices = V_template;
	MatrixXi &template_faces = F_template;

	// move template to its mean
	// Alternatively, we could just as well replace the template with one that's already centered.
	RowVector3d template_mean = template_vertices.colwise().mean();
	template_vertices.rowwise() -= template_mean;
	cout << "translated template by " << -template_mean << " to center." << endl;
	// There's no point in moving our face to its mean since the rigid alignment already includes translation.

	// scale template to match current face
	auto scale = landmark_scale(landmarks, V);
	auto template_scale = landmark_scale(template_landmarks, template_vertices);
	auto scale_diff = scale / template_scale;
	template_vertices *= scale_diff;
	cout << "scaled template by " << scale_diff << "x to match face." << endl;

	// compute optimal rigid transform
	// https://igl.ethz.ch/projects/ARAP/svd_rot.pdf

	// compute normalized landmark positions
	auto landmark_positions = igl::slice(V, landmarks, 1);
	RowVector3d mean_landmark = landmark_positions.colwise().mean();
	landmark_positions -= mean_landmark;
	auto template_landmark_positions = igl::slice(template_vertices, template_landmarks, 1);
	RowVector3d mean_template_landmark = template_landmark_positions.colwise().mean();
	template_landmark_positions -= mean_template_landmark;

	// compute optimal rotation matrix
	auto covariance = landmark_positions.transpose() * template_landmark_positions;
	auto svd = covariance.jacobiSvd(ComputeFullU | ComputeFullV);
	// The referenced paper includes a diagonal matrix of ones ending with the determinant, but that seems to always be 1 anyway, so we can ignore it.
	Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
	cout << "optimal rotation:" << endl << rotation << endl << endl;

	// compute optimal translation
	RowVector3d translation = mean_template_landmark - mean_landmark * rotation.transpose();
	cout << "optimal translation:" << translation << endl << endl;

	V *= rotation.transpose();
	V.rowwise() += translation;

	return true;
}
