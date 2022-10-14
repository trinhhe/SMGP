#include <filesystem>
#include <typeinfo>
#include <igl/unproject_onto_mesh.h>

#include "3D_keypoints.hpp"
#include "imgui.h"
#include "rigid_alignment.hpp"
#include "globals.hpp"
#include "utils.hpp"

void callback_draw_3dpoint_menu() {
    int mouse_mode_type = static_cast<int>(mouse_mode);

    if (ImGui::CollapsingHeader("Landmark Selection", ImGuiTreeNodeFlags_DefaultOpen)) {

        ImGuiStyle& style = ImGui::GetStyle();
        float spacing = style.ItemInnerSpacing.x;
        float w = ImGui::GetWindowWidth()/2.f - spacing * 2.f;

        ImGui::PushItemWidth(w);

        static const char* curr_item = face_file_names[0].c_str();
        if (ImGui::BeginCombo("Available Faces", curr_item)) {
            for (int i = 0; i < face_file_names.size(); i++) {
                bool is_selected = (curr_item == face_file_names[i]);
                if (ImGui::Selectable(face_file_names[i].c_str(), is_selected)) {
                    curr_item = face_file_names[i].c_str();

                    // reload template with landmarks
                    select_face(current_template_file);
                    V_template = V; F_template = F;
                    template_landmark_vertices = landmark_selector->get_landmark_vertices_as_vector();
                    if (template_landmark_vertices.size() == 0) {
                        cout
                            << "Warning: No landmarks found for template " << current_template_file
                            << endl
                            << "Make sure to add some landmarks to the template mesh and save them before doing alignment/warping."
                            << endl;
                    }
                    
                    // load new face
                    select_face(face_file_names[i]);
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        if (ImGui::Combo("Mouse Mode (S)", &mouse_mode_type, "SELECT\0NONE\0")) {
            mouse_mode = static_cast<MouseMode>(mouse_mode_type);
        }

        ImGui::PopItemWidth();

        if (ImGui::Checkbox("Display Template", &display_template)) {
            display_face = true; // makes no sense to display nothing, and this makes it act as a quick toggle.
        }
        if (ImGui::Checkbox("Display Face", &display_face)) {
            display_template = true;
        }
        ImGui::Checkbox("Display Landmarks", &display_landmarks);

        if (ImGui::Button("Reset landmarks (R)", ImVec2(-1, 0))) {
            landmark_selector->reset_landmarks();
        }

        if (ImGui::Button("Apply selection", ImVec2(-1, 0))) {
            landmark_selector->save_current_selection();
        }

        if (ImGui::Button("Discard selection", ImVec2(-1, 0))) {
            landmark_selector->clear_current_selection();
        }

        if (ImGui::Button("Save landmarks", ImVec2(-1, 0))) {
            cout << "writing landmarks to: " << landmarks_path(current_face_file) << endl;
            landmark_selector->write_landmarks_to_file(landmarks_path(current_face_file));

            if (current_face_file == current_template_file) {
                // update preloaded template landmarks
                template_landmark_vertices = landmark_selector->get_landmark_vertices_as_vector();
            }
        }
    }
}

bool callback_3dpoint_mouse_down(Viewer &viewer, int button, int modifier) {
    if (button == (int) Viewer::MouseButton::Right)
        return false;

    if (mouse_mode == SELECT) {
        int vi = pickVertex(viewer.current_mouse_x, viewer.current_mouse_y);
        if (vi != -1)
            landmark_selector->add_landmark(vi);
        return true;
    }
    return false;
}

bool callback_3dpoint_key_down(Viewer &wiewer, unsigned char key, int modifiers) {
    if (key == 'S') {
        mouse_mode = (mouse_mode == SELECT) ? NONE : SELECT;
        return true;
    }

    if (key == 'R') //reset
    {
        landmark_selector->reset_landmarks();
        return true;
    }

    return false;
}

bool callback_3dpoint_pre_draw(Viewer &viewer) {
    if (display_face && display_landmarks) {
        MatrixXd landmarks_pos;
        VectorXi landmarks = landmark_selector->get_landmark_vertices_as_vector();
        landmarks_positions(V, landmarks, landmarks_pos);

        const int cur_selection_idx = landmark_selector->get_current_selection_idx();
        MatrixXd colors(landmarks.size(), 3);

        for (int i = 0; i < landmarks.size(); i++) {
            if (i < cur_selection_idx)
                colors.row(i) = RowVector3d(1, 0, 0);
            else
                colors.row(i) = RowVector3d(0, 0, 1);
        }

        viewer.data().add_points(landmarks_pos, colors);
        for (int i = 0; i < landmarks.size(); i++) {
            viewer.data().add_label(landmarks_pos.row(i), std::to_string(i+1));
        }
    }

    return false;
}


int pickVertex(int mouse_x, int mouse_y) {
    int vi = -1;
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(
        Eigen::Vector2f(x, y), viewer.core.view,
        viewer.core.proj, viewer.core.viewport, V, F, fid, bc
    )) {
        bc.maxCoeff(&vi);
        vi = F(fid, vi);
    }

    return vi;
}
