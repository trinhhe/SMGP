#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <igl/local_basis.h>
#include <igl/grad.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/cotmatrix.h>


/*** insert any necessary libigl headers here ***/
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>
#include <igl/lscm.h>
#include <igl/adjacency_matrix.h>
#include <igl/sum.h>
#include <igl/diag.h>
#include <igl/speye.h>
#include <igl/repdiag.h>
#include <igl/cat.h>
#include <igl/dijkstra.h>
#include <igl/adjacency_list.h>
using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

Viewer viewer;

// vertex array, #V x3
Eigen::MatrixXd V;

// face array, #F x3
Eigen::MatrixXi F;

// UV coordinates, #V x2
Eigen::MatrixXd UV;

// face colors, #F x3
Eigen::MatrixXd colors; 

bool showingUV = false;
bool freeBoundary = false;
double TextureResolution = 10;

igl::opengl::ViewerCore temp3D;
igl::opengl::ViewerCore temp2D;

void Redraw()
{
	viewer.data().clear();

	if (!showingUV)
	{
		viewer.data().set_mesh(V, F);
		viewer.data().set_face_based(false);

    if(UV.size() != 0)
    {
      viewer.data().set_uv(TextureResolution*UV);
      viewer.data().show_texture = true;
    }
	}
	else
	{
		viewer.data().show_texture = false;
		viewer.data().set_mesh(UV, F);
	}
	
	if (colors.rows() == F.rows())
		viewer.data().set_colors(colors);
}

bool callback_mouse_move(Viewer &viewer, int mouse_x, int mouse_y)
{
	if (showingUV)
		viewer.mouse_mode = igl::opengl::glfw::Viewer::MouseMode::Translation;
	return false;
}

static void computeSurfaceGradientMatrix(SparseMatrix<double> & D1, SparseMatrix<double> & D2)
{
	MatrixXd F1, F2, F3;
	SparseMatrix<double> DD, Dx, Dy, Dz;

	igl::local_basis(V, F, F1, F2, F3);
	igl::grad(V, F, DD);

	Dx = DD.topLeftCorner(F.rows(), V.rows());
	Dy = DD.block(F.rows(), 0, F.rows(), V.rows());
	Dz = DD.bottomRightCorner(F.rows(), V.rows());

	D1 = F1.col(0).asDiagonal()*Dx + F1.col(1).asDiagonal()*Dy + F1.col(2).asDiagonal()*Dz;
	D2 = F2.col(0).asDiagonal()*Dx + F2.col(1).asDiagonal()*Dy + F2.col(2).asDiagonal()*Dz;
}
static inline void SSVD2x2(const Eigen::Matrix2d& J, Eigen::Matrix2d& U, Eigen::Matrix2d& S, Eigen::Matrix2d& V)
{
	double e = (J(0) + J(3))*0.5;
	double f = (J(0) - J(3))*0.5;
	double g = (J(1) + J(2))*0.5;
	double h = (J(1) - J(2))*0.5;
	double q = sqrt((e*e) + (h*h));
	double r = sqrt((f*f) + (g*g));
	double a1 = atan2(g, f);
	double a2 = atan2(h, e);
	double rho = (a2 - a1)*0.5;
	double phi = (a2 + a1)*0.5;

	S(0) = q + r;
	S(1) = 0;
	S(2) = 0;
	S(3) = q - r;

	double c = cos(phi);
	double s = sin(phi);
	U(0) = c;
	U(1) = s;
	U(2) = -s;
	U(3) = c;

	c = cos(rho);
	s = sin(rho);
	V(0) = c;
	V(1) = -s;
	V(2) = s;
	V(3) = c;
}

void ConvertConstraintsToMatrixForm(VectorXi indices, MatrixXd positions, Eigen::SparseMatrix<double> &C, VectorXd &d)
{
	// Convert the list of fixed indices and their fixed positions to a linear system
	// Hint: The matrix C should contain only one non-zero element per row and d should contain the positions in the correct order.
    C.resize(indices.size()*2, V.rows()*2);
    C.reserve(indices.size()*2);
    d.resize(indices.size()*2);
    vector<Triplet<double> > tripletList;
    tripletList.reserve(indices.size()*2);
    for(int i = 0; i < indices.size(); i++) {
        d(i) = positions(i,0);
        d(i+indices.size()) = positions(i,1);
        tripletList.push_back(Triplet<double>(i, indices(i), 1));
        tripletList.push_back(Triplet<double>(i+indices.size(), indices(i)+V.rows(), 1));
    }

    C.setFromTriplets(tripletList.begin(), tripletList.end());
}

void computeParameterization(int type)
{
	VectorXi fixed_UV_indices;
	MatrixXd fixed_UV_positions;

	SparseMatrix<double> A;
	VectorXd b;
	Eigen::SparseMatrix<double> C;
	VectorXd d;
	// Find the indices of the boundary vertices of the mesh and put them in fixed_UV_indices
	if (!freeBoundary)
	{
		// The boundary vertices should be fixed to positions on the unit disc. Find these position and
		// save them in the #V x 2 matrix fixed_UV_position.
        igl::boundary_loop(F, fixed_UV_indices);
        igl::map_vertices_to_circle(V, fixed_UV_indices, fixed_UV_positions);
	}
	else
	{
		// Fix two UV vertices. This should be done in an intelligent way. Hint: The two fixed vertices should be the two most distant one on the mesh.
        igl::boundary_loop(F, fixed_UV_indices);
        int first_v = fixed_UV_indices(0);
        int second_v = fixed_UV_indices(int(fixed_UV_indices.size()/2));
        fixed_UV_indices.resize(2);
        fixed_UV_indices(0) = first_v;
        fixed_UV_indices(1) = second_v;

        fixed_UV_positions.resize(2,2);
        fixed_UV_positions.row(0) = Vector2d(0,0);
        fixed_UV_positions.row(1) = Vector2d(1,0);
    }

	ConvertConstraintsToMatrixForm(fixed_UV_indices, fixed_UV_positions, C, d);


	// Find the linear system for the parameterization (1- Tutte, 2- Harmonic, 3- LSCM, 4- ARAP)
	// and put it in the matrix A.
	// The dimensions of A should be 2#V x 2#V.
	if (type == '1') {
		// Add your code for computing uniform Laplacian for Tutte parameterization
		// Hint: use the adjacency matrix of the mesh
        SparseMatrix<double> adj_matrix, uni_L;
        igl::adjacency_matrix(F, adj_matrix);
        VectorXd adj_sum;
        adj_sum = adj_matrix * VectorXd::Ones(adj_sum.cols());
        uni_L = adj_matrix - (Eigen::MatrixXd(adj_sum.asDiagonal()).sparseView());
        A.resize(2*V.rows(), 2*V.rows());
        A.reserve(uni_L.nonZeros() * 2);
        vector<Triplet<double> > tripletList;
        tripletList.reserve(uni_L.nonZeros() * 2);
        for(int i=0; i < uni_L.outerSize(); i++) {
            for(SparseMatrix<double>::InnerIterator it(uni_L,i); it; ++it){
                tripletList.push_back(Triplet<double>(it.row(), it.col(), it.value()));
                tripletList.push_back(Triplet<double>(it.row()+V.rows(), it.col()+V.rows(), it.value()));
            }
        }
        A.setFromTriplets(tripletList.begin(), tripletList.end());
        // igl::repdiag(uni_L,2,A);
        // cout << "A size: " << A.rows() << " " << A.cols() << endl;
        // cout << A << endl;
        b.setZero(2*V.rows());
	}

	if (type == '2') {
		// Add your code for computing cotangent Laplacian for Harmonic parameterization
		// Use can use a function "cotmatrix" from libIGL, but ~~~~***READ THE DOCUMENTATION***~~~~
        SparseMatrix<double> cot_L;
        igl::cotmatrix(V, F, cot_L);

        A.resize(2*V.rows(), 2*V.rows());
        A.reserve(cot_L.nonZeros() * 2);
        vector<Triplet<double> > tripletList;
        tripletList.reserve(cot_L.nonZeros() * 2);
        for(int i=0; i < cot_L.outerSize(); i++) {
            for(SparseMatrix<double>::InnerIterator it(cot_L,i); it; ++it){
                tripletList.push_back(Triplet<double>(it.row(), it.col(), it.value()));
                tripletList.push_back(Triplet<double>(it.row()+V.rows(), it.col()+V.rows(), it.value()));
            }
        }
        A.setFromTriplets(tripletList.begin(), tripletList.end());
        b.setZero(2*V.rows());
	}

	if (type == '3') {
		// Add your code for computing the system for LSCM parameterization
		// Note that the libIGL implementation is different than what taught in the tutorial! Do not rely on it!!
        VectorXd tri_areas;
        igl::doublearea(V,F,tri_areas);
        tri_areas /= 2;
        SparseMatrix<double> triangle_areas;
        triangle_areas = MatrixXd(tri_areas.asDiagonal()).sparseView();
        SparseMatrix<double> D1, D2;
        computeSurfaceGradientMatrix(D1, D2);
        SparseMatrix<double> left_top = 2*(D1.transpose()*triangle_areas*D1 + D2.transpose()*triangle_areas*D2);
        SparseMatrix<double> right_top = 2*(D2.transpose()*triangle_areas*D1 - D1.transpose()*triangle_areas*D2);
        SparseMatrix<double> left_bot= 2*(D1.transpose()*triangle_areas*D2 - D2.transpose()*triangle_areas*D1);
        SparseMatrix<double> right_bot = 2*(D1.transpose()*triangle_areas*D1 + D2.transpose()*triangle_areas*D2);
        SparseMatrix<double> temp1, temp2;
        igl::cat(2,left_top,right_top,temp1);
        igl::cat(2,left_bot,right_bot,temp2);
        igl::cat(1, temp1, temp2, A);

        b.setZero(2*V.rows());
        // cout << left_top.rows() << " " << left_top.cols() << endl;
        // cout << "D1 size: " << D1.rows() << " " << D1.cols() << endl;
        // cout << "D2 size: " << D2.rows() << " " << D2.cols() << endl;
        // cout << "#Faces: " << F.rows()  << endl;
        // cout << "#V: " << V.rows() << endl;
        // cout << "#constraints: " << fixed_UV_indices.size() << endl;
    }

	if (type == '4') {
		// Add your code for computing ARAP system and right-hand side
		// Implement a function that computes the local step first
		// Then construct the matrix with the given rotation matrices

        //DOESN'T WORK :(
        Matrix2d U, S, V;
        SparseMatrix<double> D1, D2, D1_T, D2_T;
        computeSurfaceGradientMatrix(D1, D2);
        SparseMatrix<double> D_stacked;
        D1_T = D1.transpose();
        D2_T = D2.transpose();
        igl::cat(2,D1_T, D2_T ,D_stacked);
        VectorXd tri_areas;
        igl::doublearea(V,F,tri_areas);
        tri_areas /= 2;
        SparseMatrix<double> triangle_areas;
        cout << "D_stacked " << D_stacked.rows() << " " << D_stacked.cols() << endl;
        triangle_areas = MatrixXd(tri_areas.asDiagonal()).sparseView();
        MatrixXd J_stacked = UV.transpose() * MatrixXd(D_stacked);
        cout << "J_stacked " << J_stacked.rows() << " " << J_stacked.cols() << endl;
        MatrixXd R_stacked(J_stacked.rows(), J_stacked.cols());
        for(int i = 0; i < F.rows()-1; i++) {
            assert(2*i < J_stacked.cols());
            SSVD2x2(J_stacked.block(0,2*i,2,2), U, S, V);
            // cout << "LOLOL" << endl;
            R_stacked.block(0,2*i,2,2) = U * V.transpose();
        }
        // cout << R_stacked << endl;
        //A consists also of contangent laplacian
        SparseMatrix<double> cot_L;
        igl::cotmatrix(V, F, cot_L);
        // A.resize(2*V.rows(), 2*V.rows());
        // A.reserve(cot_L.nonZeros() * 2);
        // vector<Triplet<double> > tripletList;
        // tripletList.reserve(cot_L.nonZeros() * 2);
        // for(int i=0; i < cot_L.outerSize(); i++) {
        //     for(SparseMatrix<double>::InnerIterator it(cot_L,i); it; ++it){
        //         tripletList.push_back(Triplet<double>(it.row(), it.col(), it.value()));
        //         tripletList.push_back(Triplet<double>(it.row()+V.rows(), it.col()+V.rows(), it.value()));
        //     }
        // }
        // A.setFromTriplets(tripletList.begin(), tripletList.end());
        igl::repdiag(cot_L,2,A);
        // //rightside b
        // b.resize(2*V.rows());
        b.setZero(2*V.rows());
        // cout << R_stacked.rows() << " " << R_stacked.cols() << endl;
        
	}

	// Solve the linear system.
	// Construct the system as discussed in class and the assignment sheet
	// Use igl::cat to concatenate matrices
	// Use Eigen::SparseLU to solve the system. Refer to tutorial 3 for more detail

	// The solver will output a vector
    SparseMatrix<double> zeroes(C.cols(), C.cols());
    SparseMatrix<double> temp1, temp2, K;
    SparseMatrix<double> C_T = C.transpose();
    igl::cat(2, A, C_T, temp1);
    igl::cat(2, C, zeroes, temp2);
    igl::cat(1, temp1, temp2, K);
    VectorXd Y (b.size() + d.size());
    Y << b, d;
    // cout << "K.size: " << K.rows() << " " << K.cols() << endl;
    // cout << " Y.size() " << Y.size() << endl;
    VectorXd x(2*V.rows() + C.rows());
    // cout << " x.size() " << x.size()  << endl;
    K.makeCompressed();
    SparseLU<SparseMatrix<double>, COLAMDOrdering<int> > solver;
    solver.analyzePattern(K);
    solver.factorize(K);
    x = solver.solve(Y);
	UV.resize(V.rows(), 2);
	UV.col(0) = x.head(V.rows());
	UV.col(1) = x.segment(V.rows(), V.rows());
}


bool callback_key_pressed(Viewer &viewer, unsigned char key, int modifiers) {
	switch (key) {
	case '1':
	case '2':
	case '3':
	case '4':
		computeParameterization(key);
		break;
	case '5':
			// Add your code to visualize the distortion of the triangles here
			colors = Eigen::MatrixXd(F.rows(), 3);
            // colors.setZero();
			// colors.rowwise() = Eigen::RowVector3d(255.0/255.0,228.0/255.0,58.0/255.0);
            colors.rowwise() = Eigen::RowVector3d(255, 0, 0); // 255,255,255 white
            
		break;  
	case '+':
		TextureResolution /= 2;
		break;
	case '-':
		TextureResolution *= 2;
		break;
	case ' ': // space bar -  switches view between mesh and parameterization
    if(showingUV)
    {
      temp2D = viewer.core;
      viewer.core = temp3D;
      showingUV = false;
    }
    else
    {
      if(UV.rows() > 0)
      {
        temp3D = viewer.core;
        viewer.core = temp2D;
        showingUV = true;
      }
      else { std::cout << "ERROR ! No valid parameterization\n"; }
    }
    break;
	}
	Redraw();
	return true;
}

bool load_mesh(string filename)
{
  igl::read_triangle_mesh(filename,V,F);
  Redraw();
  viewer.core.align_camera_center(V);
  showingUV = false;

  return true;
}

bool callback_init(Viewer &viewer)
{
	temp3D = viewer.core;
	temp2D = viewer.core;
	temp2D.orthographic = true;

	return false;
}

int main(int argc,char *argv[]) {
  if(argc != 2) {
    cout << "Usage ex4_bin <mesh.off/obj>" << endl;
    load_mesh("../data/cathead.obj");
  }
  else
  {
    // Read points and normals
    load_mesh(argv[1]);
  }

	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);

	menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		menu.draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("Parmaterization", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
			ImGui::Checkbox("Free boundary", &freeBoundary);

			// TODO: Add more parameters to tweak here...
            
		}
	};

  viewer.callback_key_pressed = callback_key_pressed;
  viewer.callback_mouse_move = callback_mouse_move;
  viewer.callback_init = callback_init;

  viewer.launch();
}
