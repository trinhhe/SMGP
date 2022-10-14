#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
/*** insert any necessary libigl headers here ***/
#include <igl/per_face_normals.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/slice.h>

using namespace std;
using namespace Eigen;
using Viewer = igl::opengl::glfw::Viewer;

// Input: imported points, #P x3
Eigen::MatrixXd P;

// Input: imported normals, #P x3
Eigen::MatrixXd N;

// Intermediate result: constrained points, #C x3
Eigen::MatrixXd constrained_points;

// Intermediate result: implicit function values at constrained points, #C x1
Eigen::VectorXd constrained_values;

// constraints point colors, #C x 3
Eigen::MatrixXd constrained_colors;

// Parameter: degree of the polynomial
int polyDegree = 0;

// Parameter: Wendland weight function radius (make this relative to the size of the mesh)
double wendlandRadius = 0.1;

// Parameter: grid resolution
int resolution = 20;

// Intermediate result: grid points, at which the implicit function will be evaluated, #G x3
Eigen::MatrixXd grid_points;

// Intermediate result: implicit function values at the grid points, #G x1
Eigen::VectorXd grid_values;

// Intermediate result: grid point colors, for display, #G x3
Eigen::MatrixXd grid_colors;

// Intermediate result: grid lines, for display, #L x6 (each row contains
// starting and ending point of line segment)
Eigen::MatrixXd grid_lines;

// Output: vertex array, #V x3
Eigen::MatrixXd V;

// Output: face array, #F x3
Eigen::MatrixXi F;

// Output: face normals of the reconstructed mesh, #F x3
Eigen::MatrixXd FN;

// Grid bounds: axis-aligned bounding box
Eigen::RowVector3d bb_min, bb_max;

// Bounding box dimensions
Eigen::RowVector3d dim;

//spatial index
vector< vector<int > > spatial_index;

// number of grids in 1 dimension
int grid_dim = 2;

string pathfile;
// Functions
void createGrid();
void evaluateImplicitFunc();
void getLines();
bool callback_key_down(Viewer& viewer, unsigned char key, int modifiers);
// Creates a grid_points array for the simple sphere example. The points are
// stacked into a single matrix, ordered first in the x, then in the y and
// then in the z direction. If you find it necessary, replace this with your own
// function for creating the grid.
void createGrid() {
    grid_points.resize(0, 3);
    grid_colors.resize(0, 3);
    grid_lines. resize(0, 6);
    grid_values.resize(0);
    V. resize(0, 3);
    F. resize(0, 3);
    FN.resize(0, 3);

    bb_min = P.colwise().minCoeff();
    bb_max = P.colwise().maxCoeff();
    
    dim = bb_max - bb_min;
    dim /= 10;
    bb_min -= dim/2;
    bb_max += dim/2;
    dim = bb_max - bb_min;
    // Grid spacing
    const double dx = dim[0] / (double)(resolution - 1);
    const double dy = dim[1] / (double)(resolution - 1);
    const double dz = dim[2] / (double)(resolution - 1);
    // cout << "dx, dy, dz: " << dx << " " << dy << " " << dz << endl;
    // 3D positions of the grid points -- see slides or marching_cubes.h for ordering
    grid_points.resize(resolution * resolution * resolution, 3);
    // Create each gridpoint
    for (unsigned int x = 0; x < resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                // Linear index of the point at (x,y,z)
                int index = x + resolution * (y + resolution * z);
                // 3D point at (x,y,z)
                grid_points.row(index) = bb_min + Eigen::RowVector3d(x * dx, y * dy, z * dz);
            }
        }
    }
}

double get_distance(Eigen::RowVector3d x, Eigen::RowVector3d y) {
    return (x-y).norm();
}

int get_spatial_index(RowVector3d p) {
    int idx_x, idx_y, idx_z;
    const double dx = dim[0] / (double) (grid_dim - 1);
    const double dy = dim[1] / (double) (grid_dim - 1);
    const double dz = dim[2] / (double) (grid_dim - 1);
    idx_x = floor((p(0)- bb_min(0))/dx);
    idx_y = floor((p(1)- bb_min(1))/dy);
    idx_z = floor((p(2)- bb_min(2))/dz);
    // cout << "idxyz " << idx_x << " " << idx_y << " " << idx_z << endl;
    // return idx_x + resolution * (idx_y + resolution * idx_z);
    return idx_x + grid_dim * (idx_y + grid_dim * idx_z);

}

//assign points to corresponding uniform grid cell id
void fill_spatial_index(MatrixXd points) {
    spatial_index.resize(0);
    // grid_dim = wendlandRadius;
    spatial_index.resize(grid_dim*grid_dim*grid_dim);
    bb_min = points.colwise().minCoeff();
    bb_max = points.colwise().maxCoeff();
    dim = bb_max - bb_min;
    //make bounding box slightly larger;
    dim /= 10;
    bb_min -= dim/2;
    bb_max += dim/2;
    dim = bb_max - bb_min;
    int index;
    for(int i = 0; i < points.rows(); i++) {
        index = get_spatial_index(points.row(i));
        spatial_index[index].push_back(i);
    }
    // cout << "spatial_index size " << spatial_index.size() << endl;

}

//get neighbour grid ids + grid id of p
vector<int> get_neighbour_grids(RowVector3d p) {
    vector<int> grid_neighbours;
    int idx_x, idx_y, idx_z;
    const double dx = dim[0] / (double) (grid_dim - 1);
    const double dy = dim[1] / (double) (grid_dim - 1);
    const double dz = dim[2] / (double) (grid_dim - 1);

    idx_x = floor((p(0)- bb_min(0))/dx);
    idx_y = floor((p(1)- bb_min(1))/dy);    
    idx_z = floor((p(2)- bb_min(2))/dz);
    int startX = (idx_x - 1 < 0) ? idx_x : idx_x-1;
    int startY = (idx_y - 1 < 0) ? idx_y : idx_y-1;
    int startZ = (idx_z - 1 < 0) ? idx_z : idx_z-1;
    int endX = (idx_x + 1 > grid_dim-1) ? idx_x : idx_x+1;
    int endY = (idx_y + 1 > grid_dim-1) ? idx_y : idx_y+1;
    int endZ = (idx_z + 1 > grid_dim-1) ? idx_z : idx_z+1;
    for(int i=startX; i<=endX; i++) {
        for(int j=startY; j<=startY; j++) {
            for(int k=startZ; k<=startZ; k++){
                grid_neighbours.push_back(i + grid_dim * (j + grid_dim * k));
                // if(i >= 0 && j >= 0 && k >= 0 && i < grid_dim && j < grid_dim && k < grid_dim)
                    // grid_neighbours.push_back(i + resolution * (j + resolution * k));

            }
        }
    }

    return grid_neighbours;
}

//check if nearest point with uniform grid
bool is_nearest_point(RowVector3d p_constrained, RowVector3d p_i) {
    double dist = get_distance(p_i, p_constrained);
    vector<int> grid_ids = get_neighbour_grids(p_constrained);
    for(const auto& id: grid_ids) {
        assert(id >=0);
        for(int j = 0; j < spatial_index[id].size(); j++) {
            double d = get_distance(P.row(spatial_index[id][j]), p_constrained);
            if(d < dist)
                return true;
        }
    }
    return false;
}

//bruteforce checking neighbours if point is nearest to p_i
bool is_nearest_point_brute(RowVector3d p_constrained, RowVector3d p_i) {
    double dist = get_distance(p_i, p_constrained);
    for(int i = 0; i < P.rows(); i++) {
        double d = get_distance(P.row(i), p_constrained);
        if(d < dist) 
            return true;  
    }
    return false;
}

//Wendland function
double wendland_function(double x) {
    return pow((1-x/wendlandRadius),4)*(4*x/wendlandRadius + 1);
}

RowVectorXd basis_function(RowVector3d p, int degree) {
    if(degree == 1)
        return RowVector4d(1, p[0], p[1], p[2]);
    else {
        RowVectorXd b(1,10);
        b << 1, p[0], p[1], p[2], p[0]*p[1], p[0]*p[2], p[1]*p[2],p[0]*p[0], p[1]*p[1], p[2]*p[2];
        return b;
    }
}

MatrixXd basis_matrix(MatrixXd p, int degree){
    MatrixXd b;
    if(degree == 0) {
        b.resize(p.rows(), 1);
        return b.setConstant(1);  
    }
    else if(degree == 1) {
        b.resize(p.rows(), 4);
        for(int i = 0; i < p.rows(); i++) {
            b.row(i) = RowVector4d(1, p(i,0), p(i,1), p(i,2));
        }
        return b;
    } 
    else {
        b.resize(p.rows(), 10);
        for(int i = 0; i < p.rows(); i++) {
            b(i,0) = 1;
            b(i,1) = p(i,0);
            b(i,2) = p(i,1);
            b(i,3) = p(i,2);
            b(i,4) = p(i,0) * p(i,1);
            b(i,5) = p(i,0) * p(i,2);
            b(i,6) = p(i,1) * p(i,2);
            b(i,7) = p(i,0) * p(i,0);
            b(i,8) = p(i,1) * p(i,1);
            b(i,9) = p(i,2) * p(i,2);
        }
        return b;
    }
}

// Function for explicitly evaluating the implicit function for a sphere of
// radius r centered at c : f(p) = ||p-c|| - r, where p = (x,y,z).
// This will NOT produce valid results for any mesh other than the given
// sphere.                    int cols;

// Replace this with your own function for evaluating the implicit function
// values at the grid points using MLS
void evaluateImplicitFunc() {

    fill_spatial_index(constrained_points);

    // Scalar values of the grid points (the implicit function values)
    grid_values.resize(resolution * resolution * resolution);

    for (unsigned int x = 0; x < resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                // Linear index of the point at (x,y,z)
                int index = x + resolution * (y + resolution * z);
                RowVector3d current_gridpoint = grid_points.row(index);
                
                //finding all nearby points within wendlandRadius
                vector<int> grid_neighbors = get_neighbour_grids(current_gridpoint);
                vector<int> point_neighbour_ids;
                vector<double> weights;
                for(int& id: grid_neighbors) {
                    for(int i = 0; i < spatial_index[id].size(); i++) {
                        double d = get_distance(constrained_points.row(spatial_index[id][i]), current_gridpoint);
                        if(d < wendlandRadius) {
                            point_neighbour_ids.push_back(spatial_index[id][i]);
                            weights.push_back(wendland_function(d));
                        }
                    }
                }
                if (point_neighbour_ids.empty()){
                    grid_values[index] = 50;
                }
                else {
                    VectorXi row_indices = VectorXi::Map(point_neighbour_ids.data(), point_neighbour_ids.size());
                    // VectorXi row_indices = VectorXi::Map(point_neighbour_ids.data(), 10);
                    MatrixXd points(point_neighbour_ids.size(), 3);
                    VectorXd desired_values(point_neighbour_ids.size());
                    igl::slice(constrained_points, row_indices, 1, points);
                    igl::slice(constrained_values, row_indices, 1, desired_values);
                    VectorXd proximity_weights = VectorXd::Map(weights.data(), weights.size());
                    MatrixXd b = basis_matrix(points, polyDegree);
                    b = b.array().colwise() * proximity_weights.array();
                    desired_values = desired_values.cwiseProduct(proximity_weights);
                    VectorXd c = b.fullPivHouseholderQr().solve(desired_values);
                    grid_values[index] = basis_function(grid_points.row(index), polyDegree).dot(c);
                }            
            }
        }
    }
}

// Code to display the grid lines given a grid structure of the given form.
// Assumes grid_points have been correctly assigned
// Replace with your own code for displaying lines if need be.
void getLines() {
    int nnodes = grid_points.rows();
    grid_lines.resize(3 * nnodes, 6);
    int numLines = 0;

    for (unsigned int x = 0; x<resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                int index = x + resolution * (y + resolution * z);
                if (x < resolution - 1) {
                    int index1 = (x + 1) + y * resolution + z * resolution * resolution;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
                if (y < resolution - 1) {
                    int index1 = x + (y + 1) * resolution + z * resolution * resolution;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
                if (z < resolution - 1) {
                    int index1 = x + y * resolution + (z + 1) * resolution * resolution;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
            }
        }
    }

    grid_lines.conservativeResize(numLines, Eigen::NoChange);
}



bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
    if (key == '1') {
        // Show imported points
        viewer.data().clear();
        viewer.core.align_camera_center(P);
        viewer.data().point_size = 11;
        viewer.data().add_points(P, Eigen::RowVector3d(0,0,0));
    }

    if (key == '2') {
        // Show all constraints
        viewer.data().clear();
        viewer.core.align_camera_center(P);
        // Add your code for computing auxiliary constraint points here
        // Add code for displaying all points, as above

        int n = P.rows();
        constrained_points.resize(3*n,3);
        constrained_values.resize(3*n);
        //bounding_box
        Eigen::RowVector3d bb_min, bb_max;
        bb_min = P.colwise().minCoeff();
        bb_max = P.colwise().maxCoeff();
        RowVector3d pos_constraint, neg_constraint;
        double old_eps = 0.01 * (bb_max-bb_min).norm();
        double eps = old_eps;
        auto start = chrono::high_resolution_clock::now();
        // fill_spatial_index(P);
        for(int i = 0; i < n; i++) {
            do {
                pos_constraint = P.row(i) + eps*N.row(i);
                eps /= 2;
            } while(is_nearest_point_brute(pos_constraint, P.row(i)));
            constrained_points.row(n+i) = pos_constraint;
            constrained_values(n+i) = eps*2;
            eps = old_eps;
        }

        for(int i = 0; i < n; i++) {
            do {
                neg_constraint = P.row(i) - eps*N.row(i);
                eps /= 2;
            } while(is_nearest_point_brute(neg_constraint, P.row(i)));
            constrained_points.row(2*n+i) = neg_constraint;
            constrained_values(2*n+i) = -(eps*2);
            eps = old_eps;

            //on surface constraint
            constrained_points.row(i) = P.row(i);
            constrained_values(i) = 0;
        }
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        cout << duration.count() << endl;

        //coloring
        constrained_colors.setZero(3*n,3);
        for(int i = 0;i < n; i++) {
            constrained_colors(i, 0) = 1; //red
            constrained_colors(n+i, 1) = 1; //green
            constrained_colors(2*n+i, 2) = 1; //blue       
        }
        viewer.data().point_size = 6;
        viewer.data().add_points(constrained_points, constrained_colors);
    }

    if (key == '3') {
        // Show grid points with colored nodes and connected with lines
        viewer.data().clear();
        viewer.core.align_camera_center(P);
        // Add code for creating a grid
        // Add your code for evaluating the implicit function at the grid points
        // Add code for displaying points and lines
        // You can use the following example:

        /*** begin: sphere example, replace (at least partially) with your code ***/
        // Make grid
        createGrid();

        // Evaluate implicit function
        evaluateImplicitFunc();

        // get grid lines
        getLines();

        // Code for coloring and displaying the grid points and lines
        // Assumes that grid_values and grid_points have been correctly assigned.
        grid_colors.setZero(grid_points.rows(), 3);

        // Build color map
        for (int i = 0; i < grid_points.rows(); ++i) {
            double value = grid_values(i);
            if (value < 0) {
                grid_colors(i, 1) = 1;
            }
            else {
                if (value > 0)
                    grid_colors(i, 0) = 1;
            }
        }

        // Draw lines and points
        viewer.data().point_size = 8;
        viewer.data().add_points(grid_points, grid_colors);
        viewer.data().add_edges(grid_lines.block(0, 0, grid_lines.rows(), 3),
                              grid_lines.block(0, 3, grid_lines.rows(), 3),
                              Eigen::RowVector3d(0.8, 0.8, 0.8));
        /*** end: sphere example ***/
    }

    if (key == '4') {
        // Show reconstructed mesh
        viewer.data().clear();
        // Code for computing the mesh (V,F) from grid_points and grid_values
        if ((grid_points.rows() == 0) || (grid_values.rows() == 0)) {
            cerr << "Not enough data for Marching Cubes !" << endl;
            return true;
        }
        // Run marching cubes
        igl::copyleft::marching_cubes(grid_values, grid_points, resolution, resolution, resolution, V, F);
        if (V.rows() == 0) {
            cerr << "Marching Cubes failed!" << endl;
            return true;
        }

        igl::per_face_normals(V, F, FN);
        viewer.data().set_mesh(V, F);
        viewer.data().show_lines = true;
        viewer.data().show_faces = true;
        viewer.data().set_normals(FN);
        pathfile.erase(0,8);
        string s = "../results/" + pathfile;
        cout << s;
        igl::writeOFF(s, V, F);
    }

    return true;
}

bool callback_load_mesh(Viewer& viewer,string filename)
{
  igl::readOFF(filename,P,F,N);
  callback_key_down(viewer,'1',0);
  return true;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
      cout << "Usage ex2_bin <mesh.off>" << endl;
    //   igl::readOFF("../data/sphere.off",P,F,N);
        igl::readOFF("../data/cat.off",P,F,N);
    //   igl::readOFF("../data/bunny-500.off",P,F,N);
    //   igl::readOFF("../data/bunny-1000.off",P,F,N);
    //   igl::readOFF("../data/horse.off",P,F,N);
    //   igl::readOFF("../data/hound.off",P,F,N);
    //   igl::readOFF("../data/luigi.off",P,F,N);


    }
	  else
	  {
		  // Read points and normals
		  igl::readOFF(argv[1],P,F,N);
          pathfile = argv[1];
	  }

    Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    viewer.callback_key_down = callback_key_down;

    menu.callback_draw_viewer_menu = [&]()
    {
      // Draw parent menu content
      menu.draw_viewer_menu();

      // Add new group
      if (ImGui::CollapsingHeader("Reconstruction Options", ImGuiTreeNodeFlags_DefaultOpen))
      {
        // Expose variable directly ...
        ImGui::InputInt("Resolution", &resolution, 0, 0);
        if (ImGui::Button("Reset Grid", ImVec2(-1,0)))
        {
          std::cout << "ResetGrid\n";
          // Recreate the grid
          createGrid();
          // Switch view to show the grid
          callback_key_down(viewer,'3',0);
        }

        // TODO: Add more parameters to tweak here...
        ImGui::InputDouble("wendland Radius", &wendlandRadius, 0, 0);
        ImGui::InputInt("Polydegree", &polyDegree, 0, 0);
        ImGui::InputInt("uniform grid resolution", &grid_dim, 0, 0);

      }

    };

    callback_key_down(viewer, '1', 0);

    viewer.launch();
}
