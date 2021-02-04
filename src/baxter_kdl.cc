#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <iostream>
#include <Eigen/Dense>
#include <string>

using namespace std;
using namespace Eigen;

namespace py = pybind11;

Matrix<float, 6, 7> baxter_jacobian(std::string arm, Matrix<float, 7, 1> theta) {
	  Matrix<float, 6, 7> jacobian_spatial = Matrix<float, 6, 7>::Zero();
	  
	  float theta1 = theta(0);
	  float theta2 = theta(1);
	  float theta3 = theta(2);
	  float theta4 = theta(3);
	  float theta5 = theta(4);
	  float theta6 = theta(5);
	  // float theta7 = theta(6); // Last joint angle is not needed to compute spatial Jacobian.

	  std::string left("left");
	  std::string right("right");

	  if (arm == left) {
	  	#include "jacobian_left.txt"

	  } else if (arm == right) {
	  	#include "jacobian_right.txt"
	  
	  } else {
	  	std::cout << "Please enter a valid arm name ('left' or 'right'). Returning zeros." << endl;
	  }
	  
	  return jacobian_spatial;
}

Matrix<float, 1, 7> baxter_gravity_vector(std::string arm, Matrix<float, 7, 1> theta) {
	  Matrix<float, 1, 7> G = Matrix<float, 1, 7>::Zero();
	  
	  float theta1 = theta(0);
	  float theta2 = theta(1);
	  float theta3 = theta(2);
	  float theta4 = theta(3);
	  float theta5 = theta(4);
	  float theta6 = theta(5);
	  float theta7 = theta(6);

	  std::string left("left");
	  std::string right("right");

	  if (arm == left) {
	  	#include "gravity_matrix_left.txt"

	  } else if (arm == right) {
	  	#include "gravity_matrix_right.txt"
	  
	  } else {
	  	std::cout << "Please enter a valid arm name ('left' or 'right'). Returning zeros." << endl;
	  }
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

    m.def("baxter_jacobian", &baxter_jacobian, "Computes the jacobian of the Baxter robot.");

    m.def("baxter_gravity_vector", &baxter_gravity_vector, "A function which computes the gravity vector of the Baxter robot.");

    // m.def("baxter_inertia_matrix", &baxter_inertia_matrix, "A function which computes the inertia matrix of the Baxter robot.");
}
