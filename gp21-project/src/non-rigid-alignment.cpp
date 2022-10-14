#include <Eigen/Core>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/boundary_loop.h>
#include <igl/cotmatrix.h>
#include <igl/repdiag.h>
#include <igl/cat.h>

#include <imgui/imgui.h>
#include <unordered_set>

#include "non-rigid-alignment.hpp"
#include "globals.hpp"
#include "utils.hpp"

VectorXi NonRigidAlignment::vertex_complement(VectorXi &v, int dmin, int dmax) {

    unordered_set<int> s;
    for(int i = 0; i < v.size(); ++i) {
        s.insert(v(i));
    }

    int idx = 0;
    VectorXi v_complement(dmax - dmin - v.size());
    for(int i = dmin; i < dmax; ++i) {
        if(s.find(i) == s.end()) {
            v_complement(idx++) = i;
        }
    }

    return v_complement;
}

void NonRigidAlignment::preprocess() {
    if(preprocessCalled)
        return;

    igl::boundary_loop(dstF, dst_boundary_vertices);
    igl::boundary_loop(srcF, src_boundary_vertices);

    MatrixXd dst_landmark_positions;
    igl::slice(dstV, dst_landmark_vertices, 1, dst_landmark_positions);

    MatrixXd src_boundary_positions;
    igl::slice(srcV, src_boundary_vertices, 1, src_boundary_positions);

    igl::cat(1, src_landmark_vertices, src_boundary_vertices, static_constaint_vertices);
    igl::cat(1, dst_landmark_positions, src_boundary_positions, static_constraint_positions);

    src_seen_vertices.clear();
    for(int i = 0; i < src_boundary_vertices.size(); ++i)
        src_seen_vertices.insert(src_boundary_vertices(i));
    for(int i = 0; i < src_landmark_vertices.size(); ++i)
        src_seen_vertices.insert(src_landmark_vertices(i));

    hasConverged = false;
    preprocessCalled = true;
}

void NonRigidAlignment::calculate_constraints() {
    
    dst_seen_vertices.clear();
    for(int i = 0; i < dst_boundary_vertices.size(); ++i)
        dst_seen_vertices.insert(dst_boundary_vertices(i));
    for(int i = 0; i < dst_landmark_vertices.size(); ++i)
        dst_seen_vertices.insert(dst_landmark_vertices(i));

    VectorXi threshold_vertices(srcV.rows());
    MatrixXd threshold_positions(srcV.rows(), 3);

    int dst_idx, thresh_idx = 0;
    Vector3d cur_pos, closest_pos;

    for(int i = 0; i < srcV.rows(); ++i) {
        if(src_seen_vertices.find(i) == src_seen_vertices.end()) {
            cur_pos = srcV.row(i).transpose();
            index->closest(cur_pos, closest_pos, dst_idx);

            if((cur_pos - closest_pos).squaredNorm() < threshold * threshold) {
                if((dst_seen_vertices.find(dst_idx) == dst_seen_vertices.end())) {
                    threshold_vertices(thresh_idx) = i;
                    threshold_positions.row(thresh_idx) = closest_pos.transpose();
                    thresh_idx++;
                    dst_seen_vertices.insert(dst_idx);
                }
            }
        }
    }

    threshold_vertices.conservativeResize(thresh_idx);
    threshold_positions.conservativeResize(thresh_idx, 3);
    
    igl::cat(1, static_constaint_vertices, threshold_vertices, constraint_vertices);
    igl::cat(1, static_constraint_positions, threshold_positions, constraint_positions);
}

bool NonRigidAlignment::warp_iteration() {
    if(hasConverged)
        return hasConverged;
        
    if(!preprocessCalled)
        preprocess();

    MatrixXd rhs;
    SparseMatrix<double> C, CT, L, L_ff, L_fc, M;
    
    calculate_constraints();
    igl::cotmatrix(srcV, srcF, L);

    const int n = srcV.rows();
    VectorXi free_vertices = vertex_complement(constraint_vertices, 0, n);

    igl::slice(L, free_vertices, free_vertices, L_ff);
    igl::slice(L, free_vertices, constraint_vertices, L_fc);

    MatrixXd b_f;
    MatrixXd b = L * srcV;
    igl::slice(b, free_vertices, 1, b_f);

    L_ff.makeCompressed(); // avoid copies
    SparseLU<SparseMatrix<double>> solver;
    solver.compute(L_ff);
    MatrixXd x_f = solver.solve(-L_fc * constraint_positions + b_f);

    igl::slice_into(x_f, free_vertices, 1, srcV);
    igl::slice_into(constraint_positions, constraint_vertices, 1, srcV);

    if(prevNumConstraints < constraint_vertices.size()) {
        prevNumConstraints = constraint_vertices.size();
        threshold *= 1.25;
        hasConverged = false;
    }
    else {
        hasConverged = true;
    }

    return hasConverged;
}

int NonRigidAlignment::warp() {
    bool converged = false;
    const int max_iterations = 100;
    int iterations = 0;

    while(!converged && (iterations < max_iterations)) {
        converged = warp_iteration();
        iterations++;
    }

    return iterations;
}
