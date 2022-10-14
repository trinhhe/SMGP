#ifndef _3D_KEYPOINTS__
#define _3D_KEYPOINTS__

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <igl/opengl/glfw/Viewer.h>
#include <filesystem>
#include <string>

#include "utils.hpp"

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

void callback_draw_3dpoint_menu();

bool callback_3dpoint_mouse_down(Viewer &viewer, int button, int modifier);

bool callback_3dpoint_key_down(Viewer &wiewer, unsigned char key, int modifiers);

bool callback_3dpoint_pre_draw(Viewer &viewer);

int pickVertex(int mouse_x, int mouse_y);

class LandmarkSelector {
    public:
        LandmarkSelector() {};
        ~LandmarkSelector() {};

        const vector<int> &get_landmark_vertices() const { return landmark_vertices; }
        const VectorXi get_landmark_vertices_as_vector() const { return VectorXi::Map(landmark_vertices.data(), landmark_vertices.size()); }
        const int get_current_selection_idx() const { return cur_selection_start_idx; }

        void add_landmark(const int v_id) {
            landmark_vertices.push_back(v_id);
        }

        void reset_landmarks() {
            landmark_vertices.clear();
            cur_selection_start_idx = 0;
        }

        void save_current_selection() {
            cur_selection_start_idx = landmark_vertices.size();
        }

        void clear_current_selection() {
            landmark_vertices.resize(cur_selection_start_idx);
        }

        void write_landmarks_to_file(const string &filename) {
            VectorXi lm_v(cur_selection_start_idx);
            VectorXi lm_ids(cur_selection_start_idx);
            for(int i = 0; i < cur_selection_start_idx; ++i) {
                lm_v(i) = landmark_vertices[i];
                lm_ids(i) = i + 1;
            }

            write_landmarks(filename, lm_v, lm_ids);
        }

    private:
        /// list of all vertices that have been selected as landmarks
        vector<int> landmark_vertices;

        /// index to the first non-saved landmark vertex in landmark_vertices (0 if no vertices saved)
        int cur_selection_start_idx;
};

#endif
