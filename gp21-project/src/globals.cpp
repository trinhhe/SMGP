#include "globals.hpp"
#include "3D_keypoints.hpp"
#include "non-rigid-alignment.hpp"

// Vertex array, #V x3
Eigen::MatrixXd V(0, 3);
// Face array, #F x3
Eigen::MatrixXi F(0, 3);

// Vertex array for the template, #V x3
Eigen::MatrixXd V_template(0, 3);
// Face array for the template, #F x3
Eigen::MatrixXi F_template(0, 3);
// Landmark vertices of the template face
Eigen::VectorXi template_landmark_vertices;

// Vertex array for the displayed mesh, #V x3
Eigen::MatrixXd V_display(0, 3);
// Face array for the displayed mesh, #F x3
Eigen::MatrixXi F_display(0, 3);

igl::opengl::glfw::Viewer viewer;
igl::opengl::glfw::imgui::ImGuiMenu menu;

// Landmark picker globals
MouseMode mouse_mode = NONE;
std::string path_to_faces;
std::string path_to_landmarks;
std::string path_to_rigidly_aligned_faces;
std::string path_to_warped_faces;
std::string path_to_pca_dataset;

string face_path(const string &face) {
	return path_to_faces + face + ".obj";
}

string landmarks_path(const string &face) {
	return path_to_landmarks + face + "_landmarks.txt";
}

string rigidly_aligned_face_path(const string &face) {
	return path_to_rigidly_aligned_faces + face + ".obj";
}

string warped_face_path(const string &face) {
	return path_to_warped_faces + face + ".obj";
}

std::vector<std::string> face_file_names;
std::string current_face_file;
std::string current_template_file;
bool landmarks_visibility;
std::unique_ptr<LandmarkSelector> landmark_selector;

std::unique_ptr<NonRigidAlignment> warper;
bool display_template = false;
bool display_face = true;
bool display_landmarks = true;

//PCA data
bool shouldAnimate = false;
bool shouldIncrease = true;

PCAMode mode = PCAMode::PCA_Normal;

//amount of eigenvectors to use for reconstruction
const int eigen_vecs = 10;
float eigen_coeffs[eigen_vecs];
Eigen::VectorXd average_face;
Eigen::MatrixXd A;
Eigen::VectorXd eigen_values;
Eigen::MatrixXd eigen_faces;
Eigen::MatrixXd eigen_vectors;
Eigen::MatrixXd average_face_mat;
int morph_target1, morph_target2;
float morph_coeff;

//global for face morphing later?
vector<face_data> flat_faces;
