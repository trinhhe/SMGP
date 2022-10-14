#ifndef _NON_RIGID_ALIGNMENT_HPP__
#define _NON_RIGID_ALIGNMENT_HPP__

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bounding_box_diagonal.h>

#include <memory>
#include <unordered_set>

#include "spatialindex.hpp"

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

class NonRigidAlignment {
    public:
        /**
         * Non-rigid alignment module that takes as input a source mesh and target mesh
         * and warps the source mesh towards the target mesh in a non-rigid fashion.
         * This solver minimized the laplacian energy: argmin(x') ||Lx' - Lx|| s.t. x_c = d 
         * where x are the known source vertex positions and x_c are the constraints vertices
         * constraint to positions d
         *
         * @param sV: Vertices of source mesh
         * @param sF: Faces of source mesh
         * @param dV: Vertices of target mesh
         * @param dF: Faces of target mesh
         * @param slm: landmark vertices of the source mesh
         * @param dlm: landmark vertices of the target mesh
         */
        NonRigidAlignment(MatrixXd &sV, MatrixXi &sF, MatrixXd &dV, MatrixXi &dF, const VectorXi &slm, const VectorXi &dlm) {
            srcV = sV;
            srcF = sF;
            dstV = dV;
            dstF = dF;
            src_landmark_vertices = slm;
            dst_landmark_vertices = dlm;

            bb_diag = igl::bounding_box_diagonal(srcV);
            threshold = 0.01 * bb_diag;

            index = new SpatialIndex();
            index->populate(dstV);

            hasConverged = false;
            preprocessCalled = false;
            prevNumConstraints = 0;
        };

        ~NonRigidAlignment() {
            delete index;
        };
    
        /// #CV x 3 matrix with all positions of the constrained vertices
        const MatrixXd &get_constraint_positions() const { return constraint_positions; }

        /// #Cv vector with the all constrained vertices
        const VectorXi &get_constraint_vertices() const { return constraint_vertices; }

        /// Returns the source mesh vertex positions
        inline const MatrixXd &get_srcV() const { return srcV; }

        /// Returns the source mesh faces
        inline const MatrixXi &get_srcF() const { return srcF; }

        /// Returns the target mesh vertex positions
        inline const MatrixXd &get_dstV() const { return dstV; }

        /// Returns the target mesh faces
        inline const MatrixXi &get_dstF() const { return dstF; }

        /// Sets the threshold from the user via the menu f.ex.
        void set_threshold(const double t) { threshold = clamp01(t) * bb_diag; }

        /// Performs a single non-rigid warp iteration
        bool warp_iteration();

        /**
         * Performs non-rigid warping until convergence. 
         * @return: the number of warping iterations until the system converged (capped to 100 iterations)
         * The convergence criterion is when after a waping iteration we still have the same number of constraints.
         */
        int warp();

    private:
        /// Small helper that clamps its @param t: to [0,1]
        inline double clamp01(const double t) const { return t < 0. ? 0. : (t > 1. ? 1. : t); } 

        /**
         * Given a vector v with vertex indices in {dmin, dmax - 1} it returns the set complement of v.
         * @param v: vector with indices within {dmin, dmax - 1}
         * @param dmin: minimum vertex label (mostly 0) 
         * @param dmax: 1 past maximum vertex label (mostly |V|)
         * @return: the vector with the vertices fomr {dmin, dmax - 1} not in v
         */
        VectorXi vertex_complement(VectorXi &v, int dmin, int dmax);

        /**
         * Calculates all constraints necessary for the warping process. There are 3 types of constraints.
         * (1) template landmark vertices. These are mapped to the template landmark positions.
         * (2) template boundary vertices. These stay fixed during the warping.
         * (3) template vertices which are not in (1) and (2) and are within threshold of some target mesh vertex.
         * During the calculation of (3) we avoid mapping different template vertices to the same target mesh vertex
         */
        void calculate_constraints();

        /**
         * Some constraints such as (1) and (2) from above only depend on the initial template mesh
         * and on the target mesh. These are precomputed and reused during each warping step when
         */
        void preprocess();

        MatrixXd srcV;
        MatrixXi srcF;
        MatrixXd dstV;
        MatrixXi dstF;

        VectorXi src_landmark_vertices;
        VectorXi dst_landmark_vertices;
        VectorXi src_boundary_vertices;
        VectorXi dst_boundary_vertices;

        VectorXi static_constaint_vertices;
        MatrixXd static_constraint_positions;

        unordered_set<int> src_seen_vertices;
        unordered_set<int> dst_seen_vertices;

        VectorXi constraint_vertices;
        MatrixXd constraint_positions;

        double threshold;
        double bb_diag;
        SpatialIndex *index;

        bool hasConverged;
        bool preprocessCalled;
        int prevNumConstraints;
};

#endif // _NON_RIGID_ALIGNMENT_HPP__
