#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace py = pybind11;

// Matrix<float, 6, 7> baxter_jacobian(Matrix<float, 7, 1> theta) {
// 	  Matrix<float, 6, 7> jacobian_spatial = Matrix<float, 6, 7>::Zero();
	  
// 	  float theta1 = theta(0);
// 	  float theta2 = theta(1);
// 	  float theta3 = theta(2);
// 	  float theta4 = theta(3);
// 	  float theta5 = theta(4);
// 	  float theta6 = theta(5);
// 	  float theta7 = theta(6);

// 	  #include "jacobian_temp.txt"
	  
// 	  return jacobian_spatial;
// }

Matrix<float, 1, 7> baxter_gravity_vector(Matrix<float, 7, 1> theta) {
	  Matrix<float, 1, 7> G = Matrix<float, 1, 7>::Zero();
	  
	  float theta1 = theta(0);
	  float theta2 = theta(1);
	  float theta3 = theta(2);
	  float theta4 = theta(3);
	  float theta5 = theta(4);
	  float theta6 = theta(5);
	  float theta7 = theta(6);

	  #include "gravity_matrix_temp.txt"
	  
	  return G;
}

// Matrix<float, 7, 7> baxter_inertia_matrix(Matrix<float, 7, 1> theta) {
// 	  Matrix<float, 1, 7> M = Matrix<float, 7, 7>::Zero();
	  
// 	  float theta1 = theta(0);
// 	  float theta2 = theta(1);
// 	  float theta3 = theta(2);
// 	  float theta4 = theta(3);
// 	  float theta5 = theta(4);
// 	  float theta6 = theta(5);
// 	  float theta7 = theta(6);

// 	  #include "inertia_matrix_temp.txt"
	  
// 	  return M;
// }

PYBIND11_MODULE(ee106b_baxter_kdl, m) {
    m.doc() = "Python bindings for computing dynamics matrices for the Baxter robot."; // optional module docstring

    // m.def("baxter_jacobian", &baxter_jacobian, "Computes the jacobian of the Baxter robot.");

    m.def("baxter_gravity_vector", &baxter_gravity_vector, "A function which computes the gravity vector of the Baxter robot.");

    // m.def("baxter_inertia_matrix", &baxter_inertia_matrix, "A function which computes the inertia matrix of the Baxter robot.");
}
