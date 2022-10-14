#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#ifndef MAESTRO_CHAI3D_MR_H
#define MAESTRO_CHAI3D_MR_H

// ! A bunch of helper functions ported from the modern robotics text

// takes a SE3 matrix and returns the 6x6 adjoint represenation
MatrixXd Adjoint(Matrix4d T);

// takes 3 vector exponential coordinates and returns the axis vector
// and the angle
tuple<Vector3d , double> AxisAng3(Vector3d expc3);

// takes a 6 vector exponential coordinates S*theta and returns the
// normalized screw axis and the distance traveled theta
tuple<VectorXd , double> AxisAng6(VectorXd expc6);

// returns distance of the matrix from the SE3 manifold
double DistanceToSE3(Matrix4d mat);

// returns the distance from mat to the SO3 manifold
double DistanceToSO3(Matrix3d mat);

// performs the FK in the body frame using the exponential formula
// and returns the SE3 matrix
Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist);

// performs the forward kinematics in the space frame and returns
// the SE3 matrix
Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist);

// performs the inverse kinematics in the body frame and returns
// the new theta loop
bool IKinBody(MatrixXd Blist, Matrix4d M, Matrix4d T, Matrix4d& Tsb, const VectorXd thetalist0,
           VectorXd & thetalist, double eomg = 0.005 , double ev = 0.005, const int maxIterations = 20);

// performs the inverse kinematics in the space frame and returns
// the new theta loop
bool IKinSpace(MatrixXd Slist, Matrix4d M, Matrix4d T, Matrix4d& Tsb, const VectorXd thetalist0,
          VectorXd & thetalist, double eomg = 0.005, double ev = 0.005, const int maxIterations = 20);

// finds the jacobian matrix in the body frame
MatrixXd JacobianBody(MatrixXd Blist, VectorXd thetalist);

// finds the jacobian matrix in the space frame
MatrixXd JacobianSpace(MatrixXd Slist, VectorXd thetalist);

// takes 3x3 so3 matrix and returns SO3
Matrix3d MatrixExp3(Matrix3d so3mat);

// takes 3x3 se3 matrix and returns SE3
Matrix4d MatrixExp6(Matrix4d se3mat);

// takes R and returns the 3x3 so3
Matrix3d MatrixLog3(Matrix3d R);

// takes the matrix T in SE3 and returns se3
Matrix4d MatrixLog6(Matrix4d T);

// takes a scalar and checks if it is small enough to be neglected
bool NearZero(double near);

// takes a matrix near SE3 and  projects it to SE3
Matrix4d ProjectToSE3(Matrix4d mat);

// takes a matrix near SO3 and projects to SO3
Matrix3d ProjectToSO3(Matrix3d mat);

// takes a rotation and returns the inverse
Matrix3d RotInv(Matrix3d R);

// takes a rotation and position and returns the transformation matrix
Matrix4d RpToTrans(Matrix3d R, Vector3d p);

// takes a point, a unit vector, and pitch of scre axis and returns the screw
// vector
VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h);

// takes a matrix and checks if it is close to or on the manifold SE3
bool TestIfSE3(Matrix4d mat);

// takes a matrix and checks if it is close to or on the manifold SO3
bool TestIfSO3(Matrix3d mat);

// takes a transformation matrix T and returns its inverse
Matrix4d TransInv(Matrix4d T);

// takes a transformation matrix and decomposes it
tuple<Matrix3d , Vector3d> TransToRp(Matrix4d T);

// takes a 6 vector and returns the se3 matrix
Matrix4d VecTose3(VectorXd V);

// takes a 3 vector and returns the se3 matrix
Matrix3d VecToso3(Vector3d omg);

// takes a 6 vector and returns the adjoint representation
MatrixXd adjoint(VectorXd V);

// takes a se3 matrix and returns the 6 vector
VectorXd se3ToVec(Matrix4d se3mat);

// takes so3 matrix and returns the 3 vector of angular velocity
Vector3d so3ToVec(Matrix3d so3mat);

#endif //MAESTRO_CHAI3D_MR_H
