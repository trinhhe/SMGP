#include <iostream>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
/*** insert any libigl headers here ***/
#include <igl/vertex_triangle_adjacency.h>
#include <igl/adjacency_list.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/jet.h>
#include <igl/facet_components.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/edge_topology.h>
#include <igl/barycenter.h>
#include <igl/PI.h>

using namespace std;
using Viewer = igl::opengl::glfw::Viewer;

// Vertex array, #V x3
Eigen::MatrixXd V;
// Face array, #F x3
Eigen::MatrixXi F;
// Per-face normal array, #F x3
Eigen::MatrixXd FN;
// Per-vertex normal array, #V x3
Eigen::MatrixXd VN;
// Per-corner normal array, (3#F) x3
Eigen::MatrixXd CN;
// Vectors of indices for adjacency relations
std::vector<std::vector<int>> VF, VFi, VV;
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing vertex to face relations here;
        // store in VF,VFi.
        igl::vertex_triangle_adjacency(V.rows(), F, VF, VFi);
        cout << "vertex id | id of faces" << endl;
        for (int i = 0; i < VF.size(); ++i)
        {
            cout << i << " | ";
            for (int j = 0; j < VF[i].size(); ++j)
                cout << VF[i][j] << " ";
            cout << endl;
        }
    }

    if (key == '2')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing vertex to vertex relations here:
        // store in VV.
        igl::adjacency_list(F, VV);
        cout << "vertex id | vertex neighbour ids" << endl;
        for (int i = 0; i < VV.size(); ++i)
        {
            cout << i << " | ";
            for (int j = 0; j < VV[i].size(); ++j)
                cout << VV[i][j] << " ";
            cout << endl;
        }
    }

    if (key == '3')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        FN.setZero(F.rows(), 3);
        // Add your code for computing per-face normals here: store in FN.
        igl::per_face_normals(V, F, FN);
        // Set the viewer normals.
        viewer.data().set_normals(FN);
    }

    if (key == '4')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing per-vertex normals here: store in VN.
        igl::per_vertex_normals(V, F, VN);
        // Set the viewer normals.
        viewer.data().set_normals(VN);
    }

    if (key == '5')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing per-corner normals here: store in CN.
        igl::per_corner_normals(V,F, FN, VF, 60, CN);
        //Set the viewer normals
        viewer.data().set_normals(CN);
    }

    if (key == '6')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        component_colors_per_face.setZero(F.rows(), 3);
        // Add your code for computing per-face connected components here:
        // store the component labels in cid.
        igl::facet_components(F,cid);
        // Compute colors for the faces based on components, storing them in
        // component_colors_per_face.
        igl::jet(cid, true, component_colors_per_face);
        // Set the viewer colors
        viewer.data().set_colors(component_colors_per_face);

        // number of connected components and size of each component calculation
        int nb_of_component = cid.maxCoeff()+1;
        vector<int> counts(nb_of_component);
        cout << endl;
        for(int i = 0; i < cid.size(); ++i)
            counts[cid[i]]++;
        cout << "Faces: " << F.rows() << endl;
        cout << "Total components: " << nb_of_component << endl;
        cout << "Component ID | Number of faces" << endl;
        cout << "------------ | ---------------" << endl;
        for(int i = 0; i < nb_of_component; ++i)
            cout << i << " | " << counts[i] << endl;

    }

    if (key == '7')
    {
        Eigen::MatrixXd Vout = V;
        Eigen::MatrixXi Fout = F;
        // Add your code for sqrt(3) subdivision here.

        // step1
        // stores midpoints of all faces, dim: #F x 3
        Eigen::MatrixXd M;
        igl::barycenter(V, F, M);
        Vout.conservativeResize(V.rows() + M.rows(), 3);
        Vout.bottomRows(M.rows()) = M;

        Eigen::MatrixXi F2(F.rows()*3, 3);
        //go through all faces
        for(int i = 0; i < F.rows(); ++i){
            //three new faces for each face
            for(int j = 0; j < 3; ++j){
                Eigen::RowVector3i f2; 
                f2 << F(i,j % 3), F(i,(j+1) % 3), i+V.rows();
                F2.row(i*3+j) = f2;
            }  
        }

        // step2
        Eigen::MatrixXd P(V.rows(), 3);
        P.setZero();
        std::vector<std::vector<int>> neighbours;
        igl::adjacency_list(F, neighbours);
        for(int i = 0; i < V.rows(); ++i){
            double n = neighbours[i].size();
            double a_n = (4.0-2.0*cos(2*igl::PI/n))/9.0;
            for(int j = 0; j < n; ++j){
                P.row(i) += V.row(neighbours[i][j]);
            }
            P.row(i) *= a_n/n;
            P.row(i) += (1.0-a_n)*V.row(i);
        }
        Vout.topRows(V.rows()) = P;

        // step3      
            
        //   TT   #F by #3 adjacent matrix, the element i,j is the id of the triangle
        //        adjacent to the j edge of triangle i
        //   TTi  #F by #3 adjacent matrix, the element i,j is the id of edge of the
        //        triangle TT(i,j) that is adjacent with triangle i
        Eigen::MatrixXi TT;
        Eigen::MatrixXi TTi;
        igl::triangle_triangle_adjacency(F2,TT,TTi); 
        Eigen::MatrixXi F1(F2.rows(), 3);
        vector<bool> completed_faces(F2.rows(), false);
        bool boundy_edge = false;
        int counter = 0;
        //loop over all faces
        for(int i = 0; i < F2.rows(); ++i){
            //check all 3 edges of face
            for(int j = 0; j < 3; ++j){
                //check if boundary edge
                if(TT(i, j) == -1){
                    F1.row(counter) = F2.row(i);
                    boundy_edge = true;
                    counter++;
                    continue;
                }
                    
                //check if endpoints of edge are both from P (points of original edges)
                if(F2(i, j) < V.rows() && F2(i, (j+1)%3) < V.rows() && !completed_faces[TT(i,j)] && !boundy_edge) {
                    Eigen::RowVector3i f, g;
                    f << F2(i,j), F2(TT(i,j),(TTi(i,j)+2)%3), F2(i,(j+2)%3);
                    g << F2(i, (j+1)%3), F2(i, (j+2)%3), F2(TT(i,j), (TTi(i,j)+2)%3);
                    F1.row(counter) = f;
                    counter++;
                    F1.row(counter) = g;
                    counter++;
                }
            }
            completed_faces[i] = true;
            boundy_edge = false;
        }


        // Set up the viewer to display the new mesh
        V = Vout;
        F = F1;
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
    }

    return true;
}

bool load_mesh(Viewer &viewer, string filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    igl::readOFF(filename, V, F);
    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V, F);
    return true;
}

int main(int argc, char *argv[])
{
    // Show the mesh
    Viewer viewer;
    viewer.callback_key_down = callback_key_down;

    std::string filename;
    if (argc == 2)
    {
        filename = std::string(argv[1]);
    }
    else
    {
        filename = std::string("../data/honda.off");
    }
    load_mesh(viewer, filename, V, F);

    callback_key_down(viewer, '1', 0);

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.launch();
}
