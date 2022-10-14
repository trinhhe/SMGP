#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/read_triangle_mesh.h>
#include <imgui/imgui.h>
#include <filesystem>
#include <iostream>

#include <igl/write_triangle_mesh.h>
#include <memory>

#include "callbacks.hpp"
#include "globals.hpp"
#include "utils.hpp"
#include "3D_keypoints.hpp"
#include "rigid_alignment.hpp"
#include "non-rigid-alignment.hpp"

enum ViewerMode{
    View_Preprocess,
    View_PCA
};

ViewerMode view = View_Preprocess;

int selected_face = 0;

void callback_draw_viewer_menu() {
    // Draw parent menu content
    menu.draw_viewer_menu();

    // code for additional menu items here  

}

void callback_draw_custom_window() {
    // This draws an additional menu window. 
    // Might be useful to display debugging information etc.

    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 450), ImGuiCond_FirstUseEver);
    ImGui::Begin(
        "Face Manipulation", nullptr,
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize
    );
    

    if(ImGui::RadioButton("Preprocessing Mode", view == View_Preprocess)){
        view = View_Preprocess;
    } ImGui::SameLine();
    if(ImGui::RadioButton("PCA Mode", view == View_PCA)){
        flat_faces.clear();
        shouldAnimate = false;
        view = View_PCA;
    }
    
    ImGui::Separator();
    if (view == View_Preprocess){
        callback_draw_3dpoint_menu();

        if(ImGui::CollapsingHeader("Alignment", ImGuiTreeNodeFlags_DefaultOpen)) {
            if(ImGui::Button("Reset Pipeline", ImVec2(-1,0))) {
                auto face_file = face_path(current_face_file);
                igl::read_triangle_mesh(face_file, V, F);

                auto template_file = face_path(current_template_file);
                igl::read_triangle_mesh(template_file, V_template, F_template);

                warper = nullptr;
            }

            if (ImGui::Button("Rigidly Align", ImVec2(-1,0))) {
                bool success = rigidly_align_to_template(landmark_selector->get_landmark_vertices_as_vector(), V);

                if(success) {
                    auto rigid_file = rigidly_aligned_face_path(current_face_file);
                    cout << "saving rigidly-aligned face to: " << rigid_file << endl;
                    igl::write_triangle_mesh(rigid_file, V, F);
                    
                    auto rigid_template = rigidly_aligned_face_path(current_template_file);
                    cout << "saving rigidly-aligned template to: " << rigid_template << endl;
                    igl::write_triangle_mesh(rigid_template, V_template, F_template);

                    warper = std::unique_ptr<NonRigidAlignment>(
                        new NonRigidAlignment(
                            V_template,
                            F_template,
                            V,
                            F,
                            template_landmark_vertices,
                            landmark_selector->get_landmark_vertices_as_vector()
                        )
                    );
                }
                else {
                    cerr << "rigid alignment failed" << endl;
                }
            }

            if(ImGui::Button("Warp", ImVec2(-1,0))) {
                if(!warper) {
                    warper = std::unique_ptr<NonRigidAlignment>(
                        new NonRigidAlignment(
                            V_template,
                            F_template,
                            V,
                            F,
                            template_landmark_vertices,
                            landmark_selector->get_landmark_vertices_as_vector()
                        )
                    );
                }

                bool converged = warper->warp_iteration();
                V_template = warper->get_srcV();

                if(converged) {
                    auto warped_template = warped_face_path(current_face_file);
                    cout << "warping converged. saving warped template to: " << warped_template << endl;
                    igl::write_triangle_mesh(warped_template, V_template, F_template);
                }
            }
            ImGuiStyle& style = ImGui::GetStyle();
            float spacing = style.ItemInnerSpacing.x;
            float w = ImGui::GetWindowWidth()/2.f - spacing * 2.f;

            ImGui::PushItemWidth(w);
            static double warp_threshold = 0.01;
            if(ImGui::InputDouble("Warp threshold", &warp_threshold, 0, 0, "%.4f")) {
                if(!warper) {
                    warper = std::unique_ptr<NonRigidAlignment>(
                        new NonRigidAlignment(
                            V_template,
                            F_template,
                            V,
                            F,
                            template_landmark_vertices,
                            landmark_selector->get_landmark_vertices_as_vector()
                        )
                    );
                }
                warper->set_threshold(warp_threshold);
            }

            ImGui::PopItemWidth();
        }

    } else if (view == View_PCA){

        ImGui::Text("PCA");

        ImGui::InputText("Dataset Folder", path_to_pca_dataset);

        if(ImGui::Button("Build Eigenfaces", ImVec2(-1,0))){
            //build the vector of eigenfaces
            build_eigenfaces();
        }

        ImGui::Separator();
        ImGui::Text("PCA Parameters");

        if (flat_faces.size() > 0){
            if (ImGui::RadioButton("Display Normal", mode == PCA_Normal)){ 
                mode = PCA_Normal;
            }; ImGui::SameLine();
                //ImGui::PushItemWidth(70);
            if (ImGui::RadioButton("Display Morph", mode == PCA_Morph)){
                mode = PCA_Morph;
            };

            if(mode == PCA_Normal){
                float width = ImGui::CalcItemWidth();

                ImGuiStyle& style = ImGui::GetStyle();
                float spacing = style.ItemInnerSpacing.x;
                //ImGui::PushItemWidth(width - spacing * 2.0f);
                ImGui::PushItemWidth(-1);

                static const char * select_face3 = flat_faces[0].filename.c_str();

                if(ImGui::BeginCombo("##combo3", select_face3)){
                    for(int i = 0; i < flat_faces.size(); i++){

                        face_data x = flat_faces[i];

                        const char * fn = x.filename.c_str();

                        bool is_selected = !strcmp(select_face3, fn);
                        if(ImGui::Selectable(fn, is_selected)){
                            select_face3 = strdup(fn);
                            selected_face = i;
                        }
                        if(is_selected){
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopItemWidth();
                //ImGui::SameLine();
                //ImGui::PushItemWidth(ImGui::CalcItemWidth()/2.f);
                float w = ImGui::GetWindowWidth()/2.f - spacing * 3.f;
                if(ImGui::Button("Display Face", ImVec2(w,0))){
                    int n = eigen_faces.cols() - 1;
                    for(int i = 0; i < eigen_vecs; i++){
                        VectorXd vec = eigen_faces.col(n-i);
                        eigen_coeffs[i] = A.col(selected_face).transpose() * vec;
                    }
                }
                ImGui::SameLine();
                if(ImGui::Button("Reset To Average##2", ImVec2(w,0))){
                    for(int i = 0; i < eigen_vecs; i++){
                        eigen_coeffs[i] = 0.f;
                    }
                }
                //ImGui::PopItemWidth();

                for(int i = 0; i < eigen_vecs; i++){
                    std::string label =  "Eigenface " + std::to_string(i);
                    ImGui::DragFloat(label.c_str(), &eigen_coeffs[i], 10.f);
                }
            } else if (mode == PCA_Morph){

                if (shouldAnimate){
                    if(ImGui::Button("Stop Animation", ImVec2(-1,0))){
                        shouldAnimate = false;
                        viewer.core.is_animating = false;
                    }
                } else {
                    if(ImGui::Button("Animate Morphing", ImVec2(-1,0))){
                        shouldAnimate = true;
                        viewer.core.is_animating = true;

                    }
                }


                float width = ImGui::CalcItemWidth();

                ImGuiStyle& style = ImGui::GetStyle();
                float spacing = style.ItemInnerSpacing.x;
                ImGui::PushItemWidth(width - spacing * 2.0f);

                static const char * select_face1 = flat_faces[0].filename.c_str();

                if(ImGui::BeginCombo("##combo", select_face1, ImGuiComboFlags_NoArrowButton)){
                    for(int i = 0; i < flat_faces.size(); i++){

                        face_data x = flat_faces[i];

                        const char * fn = x.filename.c_str();

                        bool is_selected = !strcmp(select_face1, fn);
                        if(ImGui::Selectable(fn, is_selected)){
                            select_face1 = strdup(fn);
                            morph_target1 = i;
                        }
                        if(is_selected){
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopItemWidth();
                ImGui::SameLine(0, spacing);
                if(ImGui::ArrowButton("##yes", ImGuiDir_Up)){
                    morph_target1--;
                    if(morph_target1 < 0)
                        morph_target1 = 0;
                    select_face1 = strdup(flat_faces[morph_target1].filename.c_str());
                }

                ImGui::SameLine(0, spacing);
                if(ImGui::ArrowButton("##no", ImGuiDir_Down)){
                    morph_target1++;
                    if(morph_target1 > flat_faces.size()-1)
                        morph_target1 = flat_faces.size()-1;
                    select_face1 = strdup(flat_faces[morph_target1].filename.c_str());
                }

                ImGui::PushItemWidth(width- spacing * 2.0f);

                static const char * select_face2 = flat_faces[0].filename.c_str();

                if(ImGui::BeginCombo("##combo2", select_face2, ImGuiComboFlags_NoArrowButton)){
                    for(int i = 0; i < flat_faces.size(); i++){

                        face_data x = flat_faces[i];

                        const char * fn = x.filename.c_str();

                        bool is_selected = !strcmp(select_face2, fn);
                        if(ImGui::Selectable(fn, is_selected)){
                            select_face2 = strdup(fn);
                            morph_target2 = i;
                        }
                        if(is_selected){
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopItemWidth();
                ImGui::SameLine(0, spacing);
                if(ImGui::ArrowButton("##yes2", ImGuiDir_Up)){
                    morph_target2--;
                    if(morph_target2 < 0)
                        morph_target2 = 0;
                    select_face2 = strdup(flat_faces[morph_target2].filename.c_str());
                }

                ImGui::SameLine(0, spacing);
                if(ImGui::ArrowButton("##no2", ImGuiDir_Down)){
                    morph_target2++;
                    if(morph_target2 > flat_faces.size()-1)
                        morph_target2 = flat_faces.size()-1;
                    select_face2 = strdup(flat_faces[morph_target2].filename.c_str());
                }
                
                //ImGui::DragInt("First Morph Target", &morph_target1, 1.f, 0, flat_faces.size()-1);
                //ImGui::DragInt("Second Morph Target", &morph_target2, 1.f, 0, flat_faces.size()-1);

                float w = ImGui::GetWindowWidth()/2.f - spacing * 2.f;

                ImGui::PushItemWidth(w);
                ImGui::DragFloat("Morphing Coefficient", &morph_coeff, 0.1f, 0.0f, 1.0f);

                ImGui::PopItemWidth();
            }
        }
    }
    ImGui::End();
}

/*void callback_draw_custom_window() {
    // This draws an additional menu window. 
    // Might be useful to display debugging information etc.

    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(220, 340), ImGuiCond_FirstUseEver);
    ImGui::Begin(
        "Debug Window", nullptr,
        ImGuiWindowFlags_NoSavedSettings
    );

    // Debug display stuff goes here

    int demo_variable = 42;
    ImGui::Text("Some debug variable: %d", demo_variable);

    ImGui::End();
}*/

// to add more callback options to these, simply use || short-circuiting (i.e. add another line to the return like `|| my_callback(viewer)`, moving the semicolon)

bool callback_pre_draw(Viewer &viewer) {
    viewer.data().clear();

    if(view == View_Preprocess){

        bool prevent_default = callback_3dpoint_pre_draw(viewer);

        if(!display_template && !display_face) {
            V_display.resize(0, 3);
            F_display.resize(0, 3);
        }

        if(!display_template && display_face) {
            V_display = V;
            F_display = F;
        }

        if(display_template && display_landmarks) {
            MatrixXd template_landmark_pos;
            landmarks_positions(V_template, template_landmark_vertices, template_landmark_pos);
            for(int i = 0; i < template_landmark_vertices.size(); i++) {
                viewer.data().add_points(template_landmark_pos.row(i), RowVector3d(0,1,0));
                viewer.data().add_label(template_landmark_pos.row(i), std::to_string(i+1));
            }
        }

        if(display_template && !display_face) {
            V_display = V_template;
            F_display = F_template;
        }

        if(display_template && display_face) {
            concat_meshes(V, F, V_template, F_template, V_display, F_display);
        }

        viewer.data().point_size = 10;
        viewer.data().set_mesh(V_display, F_display);
        viewer.core.align_camera_center(V_display, F_display);

        return prevent_default;

    } else if (view == View_PCA){
        float speed = 0.05f;
        if(shouldAnimate){
            if(shouldIncrease){
                morph_coeff += speed;
            }
            else {
                morph_coeff -= speed;
            }
            if (morph_coeff > 1.f){
                shouldIncrease = false;
                morph_coeff = 1.f;
            } else if (morph_coeff < 0.f){
                shouldIncrease = true;
                morph_coeff = 0.f;
            }
        }
        
        if (flat_faces.size() > 0){
            if (mode == PCA_Normal){
                MatrixXd new_vertices;
                V = average_face_mat;

                int n = eigen_faces.cols() - 1;
                for(int i = 0; i < eigen_vecs; i++){
                    VectorXd vec = eigen_faces.col(n-i);
                    inflate(vec, new_vertices);
                    V += new_vertices * eigen_coeffs[i];
                }
            } else if (mode == PCA_Morph){
                MatrixXd new_vertices;
                V = average_face_mat;

                int n = eigen_faces.cols() - 1;
                for(int i = 0; i < eigen_vecs; i++){
                    VectorXd vec = eigen_faces.col(n-i);
                    double w1 = A.col(morph_target1).transpose() * vec;
                    double w2 = A.col(morph_target2).transpose() * vec;

                    //cout << "w1 : " << w1 << endl;
                    //cout << "w2 : " << w2 << endl;

                    //morphing ratio
                    double m = morph_coeff; 
                    
                    inflate(vec, new_vertices);
                    V += new_vertices * (w1 - m * (w1 - w2));
                }
            }

            viewer.data().set_mesh(V, F);
            viewer.core.align_camera_center(V, F);
        }
    }
    return false; 
}

bool callback_post_draw(Viewer &viewer) {
    return false;
}

bool callback_mouse_down(Viewer &viewer, int button, int modifier) {
    return callback_3dpoint_mouse_down(viewer, button, modifier);
}

bool callback_mouse_up(Viewer &viewer, int button, int modifier) {
    return false;
}

bool callback_mouse_move(Viewer &viewer, int mouse_x, int mouse_y) {
    return false;
}

bool callback_mouse_scroll(Viewer &viewer, float delta_y) {
    return false;
}

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
    return callback_3dpoint_key_down(viewer, key, modifiers);
}

bool callback_key_up(Viewer &viewer, unsigned char key, int modifiers) {
    return false;
}
