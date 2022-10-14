#ifndef _GLOBALS_HPP__
#define _GLOBALS_HPP__

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>

#include "3D_keypoints.hpp"
#include "non-rigid-alignment.hpp"
#include "PCA.hpp"

// Vertex array, #V x3
extern Eigen::MatrixXd V;
// Face array, #F x3
extern Eigen::MatrixXi F;

// Vertex array for the template, #V x3
extern Eigen::MatrixXd V_template;
// Face array for the template, #F x3
extern Eigen::MatrixXi F_template;
// Landmark vertices of the template face
extern Eigen::VectorXi template_landmark_vertices;

// Vertex array for the template, #V x3
extern Eigen::MatrixXd V_display;
// Face array for the template, #F x3
extern Eigen::MatrixXi F_display;

extern igl::opengl::glfw::Viewer viewer;
extern igl::opengl::glfw::imgui::ImGuiMenu menu;

enum MouseMode { SELECT, NONE };
extern MouseMode mouse_mode;

extern std::string path_to_faces;
extern std::string path_to_landmarks;
extern std::string path_to_rigidly_aligned_faces;
extern std::string path_to_warped_faces;
/// should point to folder containing PCA-able data
extern std::string path_to_pca_dataset;

string face_path(const string &face);
string landmarks_path(const string &face);
string rigidly_aligned_face_path(const string &face);
string warped_face_path(const string &face);

extern std::vector<std::string> face_file_names;
extern std::string current_face_file;
extern std::string current_template_file;
extern bool landmarks_visibility;
extern std::unique_ptr<LandmarkSelector> landmark_selector;

extern std::unique_ptr<NonRigidAlignment> warper;
extern bool display_template;
extern bool display_face;
extern bool display_landmarks;

// PCA data

/// amount of eigenvectors to use for reconstruction
extern const int eigen_vecs;
extern float eigen_coeffs[];
extern Eigen::VectorXd average_face;
extern Eigen::MatrixXd A;
extern Eigen::VectorXd eigen_values;
extern Eigen::MatrixXd eigen_faces;
extern Eigen::MatrixXd eigen_vectors;
extern Eigen::MatrixXd average_face_mat;
extern int morph_target1, morph_target2;
extern float morph_coeff;

enum PCAMode{
  PCA_Normal,
  PCA_Morph
};
extern PCAMode mode;

struct face_data{
    std::string filename;
    VectorXd data;
};
//global for face morphing later?
extern vector<face_data> flat_faces;

extern bool shouldAnimate;
extern bool shouldIncrease;

#endif // _GLOBALS_HPP__
