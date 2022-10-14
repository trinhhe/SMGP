#ifndef _RIGID_ALIGNMENT_HPP__
#define _RIGID_ALIGNMENT_HPP__

#include <Eigen/Core>

bool rigidly_align_to_template(const Eigen::VectorXi &landmarks, Eigen::MatrixXd &V);

#endif // _RIGID_ALIGNMENT_HPP__
