//my imports
#include <dirent.h>
#include <igl/read_triangle_mesh.h>

#include <stdio.h>  //using these 
#include <unistd.h> //two to get cwd()

#include <cstring> //using strdup for ui display of filenames

#include "PCA.hpp"
#include "globals.hpp"
#include "utils.hpp"


using namespace Eigen;

void flatten(Eigen::MatrixXd &mat, Eigen::VectorXd &vec){
    vec.resize(mat.rows() * mat.cols());

    //map a matrix to a vector, by appending cols to eachother
    vec = Map<VectorXd>(mat.data(), mat.size());
}

void inflate(Eigen::VectorXd &vec, Eigen::MatrixXd &mat){
    int rows = vec.size() / 3;
    mat.resize(rows, 3);

    mat.col(0) = vec.head(rows);
    mat.col(1) = vec.segment(rows, rows);
    mat.col(2) = vec.tail(rows);
}

void build_eigenfaces(){
    DIR *dir;

    flat_faces.clear();

    char cwd [PATH_MAX];
    if(!getcwd(cwd, PATH_MAX)){
        cout << "Couldn't determine the current working directory, exiting." << endl;
        exit(0);
    }

    string path = cwd;
    path.append("/");
    path.append(path_to_pca_dataset);

    cout << "Analyzing files in directory: " << path << endl;

    if(dir = opendir(path.c_str())){
        struct dirent *file;
        while(file = readdir(dir)){

            // Vertex array, #V x3
            Eigen::MatrixXd V_;
            // Face array, #F x3
            Eigen::MatrixXi F_;

            string filename = path;
            filename.append("/");
            filename.append(file->d_name);

            if(igl::read_triangle_mesh(filename, V_, F_)){

                cout << "Reading file: " << file->d_name << " (size: " << V_.rows() << ")" << endl;

                face_data fd;
                VectorXd vec;
                flatten(V_, vec);

                string s = file->d_name;
                fd.filename = s.substr(0, s.find_first_of(".")); //remove the extensions for nicer display
                fd.data = vec;

                flat_faces.push_back(fd);

            } else {
                cout << "Skipping " << file->d_name << endl;
            }

            if(F_.rows() > 0) //some meshes somehow have no triangle data???
                F = F_; // Not sure about this step

        }

        //compute average face
        average_face.setZero(flat_faces[0].data.size());

        for(int i = 0; i < flat_faces.size(); i++){
            average_face += flat_faces[i].data;
        }
        average_face /= flat_faces.size();

        //build A matrix
        A.resize(flat_faces[0].data.size(), flat_faces.size());
        for(int i = 0; i < flat_faces.size(); i++){
            A.col(i) = flat_faces[i].data - average_face; //dont need to compute variance
        }

        MatrixXd L = A.transpose() * A;
        L /= L.cols();

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(L);

        eigen_values = eig.eigenvalues();
        eigen_vectors = eig.eigenvectors();
        eigen_faces = A * eigen_vectors;

        for (int i = 0; i < eigen_faces.cols(); i++){
            eigen_faces.col(i).normalize(); //needed???
        }

        //cout << "A matrix size : (" << A.rows() << ", " << A.cols() << ")" << endl;
        //cout << "eigen_vectors matrix size : (" << eigen_vectors.rows() << ", " << eigen_vectors.cols() << ")" << endl;
        //cout << "eigen_faces (A * eigen_vectors) matrix size : (" << eigen_faces.rows() << ", " << eigen_faces.cols() << ")" << endl;

        cout << "Done" << endl;

    }

    //build matrix of average face for later
    inflate(average_face, average_face_mat);

    closedir(dir);

}
