#include <iostream>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>


/*** insert any libigl headers here ***/
// #include <igl/per_vertex_normals.h>
#include <queue>
#include <math.h>
#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/adjacency_list.h>
#include <igl/fit_plane.h>
#include <igl/principal_curvature.h>
#include <igl/gaussian_curvature.h>
#include <igl/barycenter.h>

using namespace std;
using Viewer = igl::opengl::glfw::Viewer;

// Vertex array, #Vx3
Eigen::MatrixXd V;
// Face array, #Fx3
Eigen::MatrixXi F;

// Per-vertex uniform normal array, #Vx3
Eigen::MatrixXd N_uniform;
// Per-vertex area-weighted normal array, #Vx3
Eigen::MatrixXd N_area;
// Per-vertex mean-curvature normal array, #Vx3
Eigen::MatrixXd N_meanCurvature;
// Per-vertex PCA normal array, #Vx3
Eigen::MatrixXd N_PCA;
// Per-vertex quadratic fitted normal array, #Vx3
Eigen::MatrixXd N_quadraticFit;

// Per-vertex mean curvature, #Vx3
Eigen::VectorXd K_mean;
// Per-vertex Gaussian curvature, #Vx3
Eigen::VectorXd K_Gaussian;
// Per-vertex minimal principal curvature, #Vx3
Eigen::VectorXd K_min_principal;
// Per-vertex maximal principal curvature, #Vx3
Eigen::VectorXd K_max_principal;
// Per-vertex color array, #Vx3
Eigen::MatrixXd colors_per_vertex;

// Explicitely smoothed vertex array, #Vx3
Eigen::MatrixXd V_expLap;
// Bilateral smoothed vertex array, #Vx3
Eigen::MatrixXd V_bilateral;

//implicitely smoothed vertex array, #Vx3
Eigen::MatrixXd U;

//bfs for k-nearest neighbours
void bfs (const vector<vector<int>> &adj_list, vector<int> &neighbours, const int start, const int k) {
    // start -1 to not count start point as k neighbour
    int count = -1;
    vector<bool> seen(adj_list.size(), false);
    queue<int> Q;
    Q.push(start);
    // seen[start] = true;
    while(!Q.empty() && count!= k) {
        int x = Q.front();
        Q.pop();
        if(seen[x]) 
            continue;
        neighbours.push_back(x);
        seen[x] = true;
        for(const auto & i : adj_list[x])
            Q.push(i);
        count++;
    }
}

bool callback_key_down(Viewer& viewer, unsigned char key, int modifiers) {
    if (key == '1') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing uniform vertex normals here:
        // store in N_uniform
        cout << "KEY 1" << endl;
        N_uniform.setZero(V.rows(), 3);
        // face normals
        Eigen::MatrixXd FN;
        igl::per_face_normals(V, F, FN);
        for(int i = 0; i < F.rows(); i++) {
            for(int j = 0; j < 3; j++) {
                N_uniform.row(F(i,j)) +=  FN.row(i);
            }
        }
        N_uniform.rowwise().normalize();
        // Set the viewer normals.
        viewer.data().set_normals(N_uniform);
    }

    if (key == '2') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing area-weighted vertex normals here:
        // store in N_area
        cout << "KEY 2" << endl;
        N_area.setZero(V.rows(), 3);
        // face normals
        Eigen::MatrixXd FN;
        igl::per_face_normals(V, F, FN);
        // weights of faces
        Eigen::MatrixXd W(F.rows(), 1);
        W.setZero();
        for(int i = 0; i < F.rows(); i++) {
            for(int j = 0; j < 3; j ++) {
                double area,a,b,c,d;
                a = V(F(i,0), j) - V(F(i,2), j);
                b = V(F(i,1), j) - V(F(i,2), j);
                c = V(F(i,0), (j+1)%3) - V(F(i,2), (j+1)%3);
                d = V(F(i,1), (j+1)%3) - V(F(i,2), (j+1)%3);
                area = a*d - b*c;
                W(i) += area * area;
            }
        }
        W = W.array().sqrt().eval();
        for(int i = 0; i < F.rows(); i++) {
            for(int j = 0; j < 3; j++) {
                N_area.row(F(i,j)) +=  W(i) * FN.row(i);
            }
        }
        N_area.rowwise().normalize();
        // Set the viewer normals.
        viewer.data().set_normals(N_area);

    }

    if (key == '3') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing mean-curvature vertex normals here:
        // store in N_meanCurvature
        cout << "KEY 3" << endl;
        Eigen::SparseMatrix<double> L, M, Minv;
        igl::cotmatrix(V, F, L);
        igl::massmatrix(V,F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
        igl::invert_diag(M, Minv);
        // Eigen::MatrixXd HN;
        N_meanCurvature = -Minv*(L*V);
        N_meanCurvature.rowwise().normalize();
        Eigen::MatrixXd ref_N;
        igl::per_vertex_normals(V, F, ref_N);
        //check for flipped normals
        for(int i = 0; i < ref_N.rows(); i++) {
            if(0 > ref_N.row(i).dot(N_meanCurvature.row(i)))
                N_meanCurvature.row(i) *= -1;
        }
        // Set the viewer normals.
        viewer.data().set_normals(N_meanCurvature);
    }

    if (key == '4') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing PCA vertex normals here:
        // store in N_PCA
        cout << "KEY 4" << endl;
        N_PCA.setZero(V.rows(),3);
        vector<vector<int>> adj_list;
        igl::adjacency_list(F, adj_list);
        int k = 6;
        for(int i = 0; i < V.rows(); i++) {
            //store indices of neighbours
            vector<int> neighbours;
            bfs(adj_list, neighbours, i, k);
            Eigen::RowVector3d normal, dummy;
            Eigen::MatrixXd VV(neighbours.size(), 3);
            for(int j = 0; j < VV.rows(); j++) {
                VV.row(j) = V.row(neighbours[j]);
            }
            igl::fit_plane(VV, normal, dummy);
            N_PCA.row(i) = normal;
        }

        Eigen::MatrixXd ref_N;
        igl::per_vertex_normals(V, F, ref_N);
        //check for flipped normals
        for(int i = 0; i < ref_N.rows(); i++) {
            if(0 > ref_N.row(i).dot(N_PCA.row(i)))
                N_PCA.row(i) *= -1;
        }
        N_PCA.rowwise().normalize();
        
        // Set the viewer normals.
        viewer.data().set_normals(N_PCA);
    }

    if (key == '5') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing quadratic fitted vertex normals here:
        // store in N_quadraticFit
        cout << "KEY 5" << endl;
        N_quadraticFit.setZero(V.rows(),3);
        vector<vector<int>> adj_list;
        igl::adjacency_list(F, adj_list);
        int k = 20;
        for(int i = 0; i < V.rows(); i++) {
            vector<int> neighbours;
            bfs(adj_list, neighbours, i, k);
            Eigen::RowVector3d normal, origin_pca;
            Eigen::MatrixXd VV(neighbours.size(), 3);
            for(int j = 0; j < VV.rows(); j++) {
                VV.row(j) = V.row(neighbours[j]);
            }
            igl::fit_plane(VV, normal, origin_pca);
            vector<Eigen::RowVector3d> ref(3);
            Eigen::RowVector3d longest_i = Eigen::Vector3d(V.row(adj_list[i][0]));
            longest_i = longest_i - (normal * (longest_i-V.row(i)).dot(normal));
            longest_i -= Eigen::RowVector3d(V.row(i));
            longest_i.normalize();
            ref[0] = longest_i; //x-axis
            ref[1] = (normal.cross(longest_i)).normalized(); //y-axis
            ref[2] = normal; //z-axis
            Eigen::MatrixXd A(VV.rows(), 5);
            Eigen::MatrixXd b(VV.rows(), 1);
            Eigen::MatrixXd coef(5,1);
            vector<Eigen::RowVector3d> points(VV.rows());
            for(int j = 0; j < VV.rows(); j++) {
                Eigen::RowVector3d iTang = VV.row(j) - V.row(i);
                double u = iTang.dot(ref[0]);
                double v = iTang.dot(ref[1]);
                double n = iTang.dot(ref[2]);
                points.push_back(Eigen::RowVector3d(u,v,n));
                A(j, 0) = u*u;
                A(j, 1) = u*v;
                A(j, 2) = v*v;
                A(j, 3) = u;
                A(j, 4) = v;
                b(j) = n;
            }
            coef = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
            Eigen::RowVector3d tang1 = Eigen::RowVector3d(1,0,2*coef(0)*points[i](0) +coef(1)*points[i](1) + coef(3));
            Eigen::RowVector3d tang2 = Eigen::RowVector3d(0,1,2*coef(2)*points[i](1) +coef(1)*points[i](0) + coef(4));
            N_quadraticFit.row(i) = tang1.cross(tang2) + origin_pca; //????????
        }
        N_quadraticFit.rowwise().normalize();
        Eigen::MatrixXd ref_N;
        igl::per_vertex_normals(V, F, ref_N);
        //check for flipped normals
        for(int i = 0; i < ref_N.rows(); i++) {
            if(0 > ref_N.row(i).dot(N_PCA.row(i)))
                N_quadraticFit.row(i) *= -1;
        }
        // Set the viewer normals.
        viewer.data().set_normals(N_quadraticFit);
    }

    if (key == '6') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        cout << "KEY 6 " << endl;
        // Add your code for computing the discrete mean curvature
        // store in K_mean
        Eigen::MatrixXd HN;
        Eigen::SparseMatrix<double> L, M, Minv;
        igl::cotmatrix(V,F,L);
        igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT,M);
        igl::invert_diag(M, Minv);
        HN = -Minv*(L*V);
        K_mean = HN.rowwise().norm();
        // cout << K_mean.minCoeff() << "; " << K_mean.maxCoeff() << endl;
        igl::jet(K_mean,0,1, colors_per_vertex);
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '7') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        cout << "KEY 7 " << endl;
        // Add your code for computing the discrete Gaussian curvature
        // store in K_Gaussian
        // Eigen::VectorXd K;
        igl::gaussian_curvature(V, F, K_Gaussian);
        Eigen::SparseMatrix<double> M, Minv;
        igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_VORONOI, M);
        igl::invert_diag(M,Minv);
        K_Gaussian = (Minv*K_Gaussian).eval();
        // cout << K_Gaussian.minCoeff() << "; " << K_Gaussian.maxCoeff() << endl;
        igl::jet(K_Gaussian, 0, 1, colors_per_vertex);
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '8') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        cout << "KEY 8 " << endl;
        // Add your code for computing the discrete minimal principal curvature
        // store in K_min_principal
        Eigen::MatrixXd HN;
        Eigen::SparseMatrix<double> L, M, Minv;
        igl::cotmatrix(V,F,L);
        igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT,M);
        igl::invert_diag(M, Minv);
        HN = -Minv*(L*V);
        Eigen::VectorXd H = HN.rowwise().norm();

        Eigen::VectorXd G;
        igl::gaussian_curvature(V, F, G);
        G = (Minv*G).eval();

        K_min_principal = H + (H.cwiseProduct(H) - G).cwiseSqrt();
        // Set the viewer colors
        igl::jet(K_min_principal, 0, 1, colors_per_vertex);
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '9') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        cout << "KEY 9 " << endl;
        // Add your code for computing the discrete maximal principal curvature
        // store in K_max_principal
        Eigen::MatrixXd HN;
        Eigen::SparseMatrix<double> L, M, Minv;
        igl::cotmatrix(V,F,L);
        igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT,M);
        igl::invert_diag(M, Minv);
        HN = -Minv*(L*V);
        Eigen::VectorXd H = HN.rowwise().norm();

        Eigen::VectorXd G;
        igl::gaussian_curvature(V, F, G);
        G = (Minv*G).eval();

        K_max_principal = H - (H.cwiseProduct(H) - G).cwiseSqrt();
        // Set the viewer colors
        igl::jet(K_max_principal, 0, 1, colors_per_vertex);
        viewer.data().set_colors(colors_per_vertex);
    }
    //explicit euler
    if (key == 'E' || key == 'R') {
        // Add your code for computing explicit Laplacian smoothing here:
        // store the smoothed vertices in V_expLap
        cout << "KEY E " << endl;
        bool cotang_weights = true;
        if(key == 'E')
            V_expLap = V;
        if(key == 'R') {
            double delta = 0.000001;
            Eigen::SparseMatrix<double> L;
            if(cotang_weights){
                //cotang laplacian
                igl::cotmatrix(V,F,L);
            } else {
                Eigen::SparseMatrix<double> A;
                igl::adjacency_matrix(F,A);
                Eigen::VectorXd Asum;
                Asum = A * Eigen::VectorXd::Ones(A.cols());
                //uniform laplacian
                L = A - (Eigen::MatrixXd(Asum.asDiagonal()).sparseView());
            }
            Eigen::SparseMatrix<double> M;
            igl::massmatrix(V_expLap,F,igl::MASSMATRIX_TYPE_VORONOI,M);
            // Solve (M-delta*L) V_expLap = M*V_expLap FALSE
            const auto & S = M;
            Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
            assert(solver.info() == Eigen::Success);
            V_expLap = solver.solve((M+delta*L)*V_expLap).eval();
            // Compute centroid and subtract (also important for numerics)
            Eigen::VectorXd dblA;
            igl::doublearea(V_expLap,F,dblA);
            double area = 0.5*dblA.sum();
            Eigen::MatrixXd BC;
            igl::barycenter(V_expLap,F,BC);
            Eigen::RowVector3d centroid(0,0,0);
            for(int i = 0;i<BC.rows();i++)
            {
            centroid += 0.5*dblA(i)/area*BC.row(i);
            }
            V_expLap.rowwise() -= centroid;
            // Normalize to unit surface area (important for numerics)
            V_expLap.array() /= sqrt(area);
        }
        // viewer.data().clear();
        // viewer.data().set_mesh(V_expLap, F);
        // // viewer.data().compute_normals();
        // viewer.core.align_camera_center(V_expLap,F);
        viewer.data().set_vertices (V_expLap);
        // viewer.data().compute_normals();
        viewer.core.align_camera_center(V_expLap,F);
    }
    //implicit euler from tutorial 205_Laplacian
    if(key == 'J' || key == 'K') {
        cout << "KEY J" << endl;
        bool cotang_weights = true;
        if(key == 'J')
            U = V;
        if(key == 'K') {
            double delta = 0.001;
            Eigen::SparseMatrix<double> L;
            if(cotang_weights){
                //cotang laplacian
                igl::cotmatrix(V,F,L);
            } else {
                Eigen::SparseMatrix<double> A;
                igl::adjacency_matrix(F,A);
                Eigen::VectorXd Asum;
                Asum = A * Eigen::VectorXd::Ones(A.cols());
                //uniform laplacian
                L = A - (Eigen::MatrixXd(Asum.asDiagonal()).sparseView());
            }
            Eigen::SparseMatrix<double> M;
            igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_VORONOI,M);
            // Solve (M-delta*L) U = M*U
            const auto & S = (M - delta*L);
            Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
            assert(solver.info() == Eigen::Success);
            U = solver.solve(M*U).eval();
            // Compute centroid and subtract (also important for numerics)
            Eigen::VectorXd dblA;
            igl::doublearea(U,F,dblA);
            double area = 0.5*dblA.sum();
            Eigen::MatrixXd BC;
            igl::barycenter(U,F,BC);
            Eigen::RowVector3d centroid(0,0,0);
            for(int i = 0;i<BC.rows();i++)
            {
            centroid += 0.5*dblA(i)/area*BC.row(i);
            }
            U.rowwise() -= centroid;
            // Normalize to unit surface area (important for numerics)
            U.array() /= sqrt(area);
        }
        // viewer.data().clear();
        // viewer.data().set_mesh(U, F);
        // // viewer.data().compute_normals();
        // viewer.core.align_camera_center(U,F);
        viewer.data().set_vertices (U);
        // viewer.data().compute_normals();
        viewer.core.align_camera_center(U,F);

    }

    if (key == 'B' || key == 'N') {
        // Add your code for computing bilateral smoothing here:
        // store the smoothed vertices in V_bilateral
        if(key == 'B')
            V_bilateral = V;
        if(key == 'N') {
            // V_bilateral.setZero(V.rows(), 3);
            Eigen::MatrixXd normals;
            igl::per_vertex_normals(V_bilateral, F, normals);
            vector<vector<int>> adj_list;
            igl::adjacency_list(F, adj_list);
            int k = 10;
            double sigma_c, sigma_s;
            sigma_c = 4;
            for(int i = 0; i < V.rows(); i++) {
                vector<int> neighbours;
                //knn
                bfs(adj_list, neighbours, i, k);
                double sum = 0;
                double normalizer = 0;
                Eigen::VectorXd std_dev(neighbours.size());
                //calculate sigma_s
                double a,b,c,d;
                a = normals.row(i)[0];
                b = normals.row(i)[1];
                c = normals.row(i)[2];
                d = -normals.row(i)[0]*V_bilateral.row(i)[0] - normals.row(i)[1]*V_bilateral.row(i)[1] - normals.row(i)[2]*V_bilateral.row(i)[2];
                for(int j = 0; j < neighbours.size(); j++) {
                    std_dev(j) = abs(a*V_bilateral.row(neighbours[j])[0] + b*V_bilateral.row(neighbours[j])[1] + c*V_bilateral.row(neighbours[j])[2] + d) / sqrt(a*a + b*b + c*c);
                }
                double mean = std_dev.sum()/neighbours.size();
                std_dev = (std_dev.array() - mean).square();
                sigma_s = std_dev.sum()/neighbours.size();
                sigma_s = sqrt(sigma_s);
                // cout << "neighbours: " << neighbours.size() << endl;
                // cout << "sig_dev " << endl << std_dev << endl;
                //denoisepoint algorithm
                for(int j = 0; j < neighbours.size(); j++) {
                    double t = (V_bilateral.row(i) - V_bilateral.row(neighbours[j])).norm();
                    double h = normals.row(i).dot(V_bilateral.row(i) - V_bilateral.row(neighbours[j]));
                    double wc = exp(-(t*t) / (2*sigma_c*sigma_c));
                    double ws = exp(-(h*h) / (2*sigma_s*sigma_s));
                    sum += (ws * wc) * h;
                    normalizer += wc * ws;
                }
                V_bilateral.row(i) = V_bilateral.row(i) - normals.row(i)*(sum/normalizer);
            }

            Eigen::VectorXd dblA;
            igl::doublearea(V_bilateral, F, dblA);
            double area = 0.5*dblA.sum();
            Eigen::MatrixXd BC;
            igl::barycenter(V_bilateral, F, BC);
            Eigen::RowVector3d centroid(0,0,0);
            for(int i = 0; i < BC.rows(); i++) {
                centroid += 0.5*dblA(i)/area*BC.row(i);
            }
            V_bilateral.rowwise() -= centroid;
            V_bilateral.array() /= sqrt(area);
        }
        viewer.data().set_vertices (V_bilateral);
        // viewer.data().compute_normals();
        viewer.core.align_camera_center(V_bilateral,F);
        // viewer.data().clear();
        // viewer.data().set_mesh(V_bilateral, F);
    }

    return true;
}

bool load_mesh(Viewer& viewer,string filename, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
  igl::read_triangle_mesh(filename,V,F);
  viewer.data().clear();
  viewer.data().set_mesh(V,F);
  viewer.data().compute_normals();
  viewer.core.align_camera_center(V, F);
  return true;
}

int main(int argc, char *argv[]) {
    // Show the mesh
    Viewer viewer;
    viewer.callback_key_down = callback_key_down;
    
    std::string filename;
    if (argc == 2) {
        filename = std::string(argv[1]);
    }
    else {
        filename = std::string("../data/cow.obj");
    }
    load_mesh(viewer,filename,V,F);

    callback_key_down(viewer, '1', 0);

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    
    viewer.launch();
}
