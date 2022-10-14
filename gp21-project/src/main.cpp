#include <cstdlib>
#include <filesystem>
#include <iostream>

#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

#include "globals.hpp"
#include "callbacks.hpp"
#include "utils.hpp"
#include "3D_keypoints.hpp"

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;


int main(int argc, char *argv[]) {
    landmark_selector = std::unique_ptr<LandmarkSelector>(new LandmarkSelector());
    landmarks_visibility = true;

    path_to_faces = "../data/faces_raw/";
    path_to_landmarks = "../data/landmarks/";
    path_to_rigidly_aligned_faces = "../data/faces_rigid/";
    path_to_warped_faces = "../data/faces_warped/";
    path_to_pca_dataset = "../../data/aligned_faces_example/example2";

    if (argc == 2) {
        current_template_file = argv[1];
    } else {
        //current_template_file = "template";
        //current_template_file = "headtemplate_noneck";
        current_template_file = "headtemplate_noneck_lesshead_4k";
    }

    // read all face files into face_file_names and sort them lexicographically
    for (const auto &f : std::filesystem::directory_iterator(path_to_faces)) {
        string fname = f.path().string().substr(path_to_faces.length());
        if (fname[0] == '.') continue; // skip hidden files
        face_file_names.push_back(fname.substr(0, fname.size() - 4));
    }
    std::sort(face_file_names.begin(), face_file_names.end());

    // find, set up, and select template
    auto template_it = std::find(face_file_names.begin(), face_file_names.end(), current_template_file);
    if (template_it == face_file_names.end()) {
        cout << "Fatal Error! Could not find specified template at " << face_path(current_template_file) << endl;
        exit(1);
    }

    // move template to front
    std::rotate(face_file_names.begin(), template_it, std::next(template_it));

    // load template and store its details
    select_face(current_template_file);
    V_template = V;
    F_template = F;
    template_landmark_vertices = landmark_selector->get_landmark_vertices_as_vector();
    if (template_landmark_vertices.size() == 0) {
        cout
            << "Warning: No landmarks found for template " << current_template_file
            << endl
            << "Make sure to add some landmarks to the template mesh and save them before doing alignment/warping."
            << endl;
    }

    // --- set up viewer ---

    // add the menu to the viewer
    viewer.plugins.push_back(&menu);

    // set custom menu windows
    menu.callback_draw_viewer_menu = callback_draw_viewer_menu;
    menu.callback_draw_custom_window = callback_draw_custom_window;

    // set event callbacks
    viewer.callback_pre_draw = callback_pre_draw;
    viewer.callback_post_draw = callback_post_draw;
    viewer.callback_mouse_down = callback_mouse_down;
    viewer.callback_mouse_up = callback_mouse_up;
    viewer.callback_mouse_move = callback_mouse_move;
    viewer.callback_mouse_scroll = callback_mouse_scroll;
    viewer.callback_key_down = callback_key_down;
    viewer.callback_key_up = callback_key_up;

    // better rotation type by default
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);

    // launch the viewer
    viewer.launch();
}
