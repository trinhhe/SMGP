#include <cstdlib>
#include <fstream>
#include <utility>
#include <vector>
#include <iostream>
#include <igl/read_triangle_mesh.h>

#include "utils.hpp"
#include "globals.hpp"

using namespace std;

void read_landmarks(const string &filename, VectorXi &lm_v) {
    vector<pair<int,int>> data;
    
    ifstream infile;
    infile.open(filename);

    int vertex_id, landmark_id;
    while(infile >> vertex_id >> landmark_id) {
        data.push_back(make_pair(vertex_id, landmark_id));
    }

    infile.close();

    // sort the landmarks according to landmark id in ascending order
    std::sort(data.begin(), data.end(), [](pair<int,int> &a, pair<int,int> &b) {
        return a.second < b.second;
    });

    lm_v.resize(data.size());

    for(int i = 0; i < data.size(); ++i) {
        lm_v(i) = data[i].first;
    }
}

void write_landmarks(const string &filename, VectorXi &lm_v, VectorXi &lm_ids) {
    assert(lm_v.size() == lm_ids.size());

    ofstream outfile;
    outfile.open(filename);

    for(int i = 0; i < lm_v.size(); ++i) {
        outfile << lm_v(i) << " " << lm_ids(i) << endl;
    }

    outfile.close();
}

void select_face(std::string &face_file) {
    current_face_file = face_file;

    igl::read_triangle_mesh(face_path(face_file), V, F);

    landmark_selector->reset_landmarks();

    auto landmarks_file = landmarks_path(current_face_file);
    if (std::filesystem::exists(landmarks_file)) {
        cout << "found existing landmarks file: " << landmarks_file << endl;
        VectorXi landmarks;
        read_landmarks(landmarks_file, landmarks);

        for (int i = 0; i < landmarks.size(); i++) {
            landmark_selector->add_landmark(landmarks(i));
        }
        landmark_selector->save_current_selection();
    }
}

void concat_meshes(MatrixXd &V1, MatrixXi &F1, MatrixXd &V2, MatrixXi &F2, MatrixXd &V, MatrixXi &F) {
    V.resize(V1.rows() + V2.rows(), 3);
    F.resize(F1.rows() + F2.rows(), 3);

    V << V1, V2;
    F << F1, (F2 + MatrixXi::Constant(F2.rows(), 3, V1.rows()));
}

void landmarks_positions(Eigen::MatrixXd &V, Eigen::VectorXi &landmarks, Eigen::MatrixXd &positions) {
    positions.resize(landmarks.size(), 3);
    for (int i = 0; i < landmarks.size(); i++) {
        positions.row(i) = V.row(landmarks[i]);
    }
}
