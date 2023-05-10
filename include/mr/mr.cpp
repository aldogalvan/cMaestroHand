#include "mr.h"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;
# define M_PI           3.14159265358979323846  /* pi */

namespace mr {

    /* Function: Find if the value is negligible enough to consider 0
     * Inputs: value to be checked as a double
     * Returns: Boolean of true-ignore or false-can't ignore
     */
    bool NearZero(const double val) {
        return (std::abs(val) < .00001);
    }

    /*
     * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
     * Input: VectorXd (6x1)
     * Output: MatrixXd (6x6)
     * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
     */
    MatrixXd ad(VectorXd V) {
        Matrix3d omgmat = VecToso3(Vector3d(V(0), V(1), V(2)));

        MatrixXd result(6, 6);
        result.topLeftCorner<3, 3>() = omgmat;
        result.topRightCorner<3, 3>() = Matrix3d::Zero(3, 3);
        result.bottomLeftCorner<3, 3>() = VecToso3(Vector3d(V(3), V(4), V(5)));
        result.bottomRightCorner<3, 3>() = omgmat;
        return result;
    }

    /* Function: Returns a normalized version of the input vector
     * Input: MatrixXd
     * Output: MatrixXd
     * Note: MatrixXd is used instead of VectorXd for the case of row vectors
     * 		Requires a copy
     *		Useful because of the MatrixXd casting
     */
    MatrixXd Normalize(MatrixXd V) {
        V.normalize();
        return V;
    }


    /* Function: Returns the skew symmetric matrix representation of an angular velocity vector
     * Input: Vector3d 3x1 angular velocity vector
     * Returns: MatrixXd 3x3 skew symmetric matrix
     */
    Matrix3d VecToso3(const Vector3d &omg) {
        Matrix3d m_ret;
        m_ret << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
        return m_ret;
    }


    /* Function: Returns angular velocity vector represented by the skew symmetric matrix
     * Inputs: MatrixXd 3x3 skew symmetric matrix
     * Returns: Vector3d 3x1 angular velocity
     */
    Vector3d so3ToVec(const MatrixXd &so3mat) {
        Vector3d v_ret;
        v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return v_ret;
    }


    /* Function: Translates an exponential rotation into it's individual components
     * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
     *				and the angle of rotation)
     * Returns: The axis and angle of rotation as [x, y, z, theta]
     */
    Vector4d AxisAng3(const Vector3d &expc3) {
        Vector4d v_ret;
        v_ret << Normalize(expc3), expc3.norm();
        return v_ret;
    }


    /* Function: Translates an exponential rotation into a rotation matrix
     * Inputs: exponenential representation of a rotation
     * Returns: Rotation matrix
     */
    Matrix3d MatrixExp3(const Matrix3d &so3mat) {
        Vector3d omgtheta = so3ToVec(so3mat);

        Matrix3d m_ret = Matrix3d::Identity();
        if (NearZero(so3mat.norm())) {
            return m_ret;
        } else {
            double theta = (AxisAng3(omgtheta))(3);
            Matrix3d omgmat = so3mat * (1 / theta);
            return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
        }
    }


    /* Function: Computes the matrix logarithm of a rotation matrix
     * Inputs: Rotation matrix
     * Returns: matrix logarithm of a rotation
     */
    Matrix3d MatrixLog3(const Matrix3d &R) {
        double acosinput = (R.trace() - 1) / 2.0;
        MatrixXd m_ret = MatrixXd::Zero(3, 3);
        if (acosinput >= 1)
            return m_ret;
        else if (acosinput <= -1) {
            Vector3d omg;
            if (!NearZero(1 + R(2, 2)))
                omg = (1.0 / std::sqrt(2 * (1 + R(2, 2)))) * Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
            else if (!NearZero(1 + R(1, 1)))
                omg = (1.0 / std::sqrt(2 * (1 + R(1, 1)))) * Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
            else
                omg = (1.0 / std::sqrt(2 * (1 + R(0, 0)))) * Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
            m_ret = VecToso3(M_PI * omg);
            return m_ret;
        } else {
            double theta = std::acos(acosinput);
            m_ret = theta / 2.0 / sin(theta) * (R - R.transpose());
            return m_ret;
        }
    }

    /* Function: Combines a rotation matrix and position vector into a single
     * 				Special Euclidian Group (SE3) homogeneous transformation matrix
     * Inputs: Rotation Matrix (R), Position Vector (p)
     * Returns: Matrix of T = [ [R, p],
     *						    [0, 1] ]
     */
    MatrixXd RpToTrans(const Matrix3d &R, const Vector3d &p) {
        MatrixXd m_ret(4, 4);
        m_ret << R, p,
                0, 0, 0, 1;
        return m_ret;
    }


    /* Function: Separates the rotation matrix and position vector from
     *				the transfomation matrix representation
     * Inputs: Homogeneous transformation matrix
     * Returns: std::vector of [rotation matrix, position vector]
     */
    std::vector<MatrixXd> TransToRp(const MatrixXd &T) {
        std::vector<MatrixXd> Rp_ret;
        Matrix3d R_ret;
        // Get top left 3x3 corner
        R_ret = T.block<3, 3>(0, 0);

        Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

        Rp_ret.push_back(R_ret);
        Rp_ret.push_back(p_ret);

        return Rp_ret;
    }


    /* Function: Translates a spatial velocity vector into a transformation matrix
     * Inputs: Spatial velocity vector [angular velocity, linear velocity]
     * Returns: Transformation matrix
     */
    MatrixXd VecTose3(const VectorXd &V) {
        // Separate angular (exponential representation) and linear velocities
        Vector3d exp(V(0), V(1), V(2));
        Vector3d linear(V(3), V(4), V(5));

        // Fill in values to the appropriate parts of the transformation matrix
        MatrixXd m_ret(4, 4);
        m_ret << VecToso3(exp), linear,
                0, 0, 0, 0;

        return m_ret;
    }


    /* Function: Translates a transformation matrix into a spatial velocity vector
     * Inputs: Transformation matrix
     * Returns: Spatial velocity vector [angular velocity, linear velocity]
     */
    VectorXd se3ToVec(const MatrixXd &T) {
        VectorXd m_ret(6);
        m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

        return m_ret;
    }


    /* Function: Provides the adjoint representation of a transformation matrix
     *			 Used to change the frame of reference for spatial velocity vectors
     * Inputs: 4x4 Transformation matrix SE(3)
     * Returns: 6x6 Adjoint Representation of the matrix
     */
    MatrixXd Adjoint(const MatrixXd &T) {
        std::vector<MatrixXd> R = TransToRp(T);
        MatrixXd ad_ret(6, 6);
        ad_ret = MatrixXd::Zero(6, 6);
        MatrixXd zeroes = MatrixXd::Zero(3, 3);
        ad_ret << R[0], zeroes,
                VecToso3(R[1]) * R[0], R[0];
        return ad_ret;
    }


    /* Function: Rotation expanded for screw axis
     * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
     * Returns: 6x6 Matrix representing the rotation
     */
    MatrixXd MatrixExp6(const MatrixXd &se3mat) {
        // Extract the angular velocity vector from the transformation matrix
        Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
        Vector3d omgtheta = so3ToVec(se3mat_cut);

        MatrixXd m_ret(4, 4);

        // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
        //									[	0	 ,		1		   ]]
        if (NearZero(omgtheta.norm())) {
            // Reuse previous variables that have our required size
            se3mat_cut = MatrixXd::Identity(3, 3);
            omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
            m_ret << se3mat_cut, omgtheta,
                    0, 0, 0, 1;
            return m_ret;
        }
            // If not negligible, MR page 105
        else {
            double theta = (AxisAng3(omgtheta))(3);
            Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
            Matrix3d expExpand = MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat +
                                 ((theta - std::sin(theta)) * (omgmat * omgmat));
            Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
            Vector3d GThetaV = (expExpand * linear) / theta;
            m_ret << MatrixExp3(se3mat_cut), GThetaV,
                    0, 0, 0, 1;
            return m_ret;
        }

    }

    MatrixXd MatrixLog6(const MatrixXd &T) {
        MatrixXd m_ret(4, 4);
        auto rp = mr::TransToRp(T);
        Matrix3d omgmat = MatrixLog3(rp.at(0));
        Matrix3d zeros3d = Matrix3d::Zero(3, 3);
        if (NearZero(omgmat.norm())) {
            m_ret << zeros3d, rp.at(1),
                    0, 0, 0, 0;
        } else {
            double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
            Matrix3d logExpand1 = MatrixXd::Identity(3, 3) - omgmat / 2.0;
            Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2) * omgmat * omgmat / theta;
            Matrix3d logExpand = logExpand1 + logExpand2;
            m_ret << omgmat, logExpand * rp.at(1),
                    0, 0, 0, 0;
        }
        return m_ret;
    }


    /* Function: Compute end effector frame (used for current spatial position calculation)
     * Inputs: Home configuration (position and orientation) of end-effector
     *		   The joint screw axes in the space frame when the manipulator
     *             is at the home position
     * 		   A list of joint coordinates.
     * Returns: Transfomation matrix representing the end-effector frame when the joints are
     *				at the specified coordinates
     * Notes: FK means Forward Kinematics
     */
    MatrixXd FKinSpace(const MatrixXd &M, const MatrixXd &Slist, const VectorXd &thetaList) {
        MatrixXd T = M;
        for (int i = (thetaList.size() - 1); i > -1; i--) {
            T = MatrixExp6(VecTose3(Slist.col(i) * thetaList(i))) * T;
        }
        return T;
    }

    /*
     * Function: Compute end effector frame (used for current body position calculation)
     * Inputs: Home configuration (position and orientation) of end-effector
     *		   The joint screw axes in the body frame when the manipulator
     *             is at the home position
     * 		   A list of joint coordinates.
     * Returns: Transfomation matrix representing the end-effector frame when the joints are
     *				at the specified coordinates
     * Notes: FK means Forward Kinematics
     */
    MatrixXd FKinBody(const MatrixXd &M, const MatrixXd &Blist, const VectorXd &thetaList) {
        MatrixXd T = M;
        for (int i = 0; i < thetaList.size(); i++) {
            T = T * MatrixExp6(VecTose3(Blist.col(i) * thetaList(i)));
        }
        return T;
    }


    /* Function: Gives the space Jacobian
     * Inputs: Screw axis in home position, joint configuration
     * Returns: 6xn Spatial Jacobian
     */
    MatrixXd JacobianSpace(const MatrixXd &Slist, const MatrixXd &thetaList) {
        MatrixXd Js = Slist;
        MatrixXd T = MatrixXd::Identity(4, 4);
        VectorXd sListTemp(Slist.col(0).size());
        for (int i = 1; i < thetaList.size(); i++) {
            sListTemp << Slist.col(i - 1) * thetaList(i - 1);
            T = T * MatrixExp6(VecTose3(sListTemp));
            // std::cout << "array: " << sListTemp << std::endl;
            Js.col(i) = Adjoint(T) * Slist.col(i);
        }

        return Js;
    }

    /*
     * Function: Gives the body Jacobian
     * Inputs: Screw axis in BODY position, joint configuration
     * Returns: 6xn Bobdy Jacobian
     */
    MatrixXd JacobianBody(const MatrixXd &Blist, const MatrixXd &thetaList) {
        MatrixXd Jb = Blist;
        MatrixXd T = MatrixXd::Identity(4, 4);
        VectorXd bListTemp(Blist.col(0).size());
        for (int i = thetaList.size() - 2; i >= 0; i--) {
            bListTemp << Blist.col(i + 1) * thetaList(i + 1);
            T = T * MatrixExp6(VecTose3(-1 * bListTemp));
            // std::cout << "array: " << sListTemp << std::endl;
            Jb.col(i) = Adjoint(T) * Blist.col(i);
        }
        return Jb;
    }


    MatrixXd TransInv(const MatrixXd &transform) {
        auto rp = mr::TransToRp(transform);
        auto Rt = rp.at(0).transpose();
        auto t = -(Rt * rp.at(1));
        MatrixXd inv(4, 4);
        inv = MatrixXd::Zero(4, 4);
        inv.block(0, 0, 3, 3) = Rt;
        inv.block(0, 3, 3, 1) = t;
        inv(3, 3) = 1;
        return inv;
    }

    MatrixXd RotInv(const MatrixXd &rotMatrix) {
        return rotMatrix.transpose();
    }

    VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h) {
        VectorXd axis(6);
        axis.segment(0, 3) = s;
        axis.segment(3, 3) = q.cross(s) + (h * s);
        return axis;
    }

    VectorXd AxisAng6(const VectorXd &expc6) {
        VectorXd v_ret(7);
        double theta = Vector3d(expc6(0), expc6(1), expc6(2)).norm();
        if (NearZero(theta))
            theta = Vector3d(expc6(3), expc6(4), expc6(5)).norm();
        v_ret << expc6 / theta, theta;
        return v_ret;
    }

    MatrixXd ProjectToSO3(const MatrixXd &M) {
        JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullV);
        MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0)
            // In this case the result may be far from M; reverse sign of 3rd column
            R.col(2) *= -1;
        return R;
    }

    MatrixXd ProjectToSE3(const MatrixXd &M) {
        Matrix3d R = M.block<3, 3>(0, 0);
        Vector3d t = M.block<3, 1>(0, 3);
        MatrixXd T = RpToTrans(ProjectToSO3(R), t);
        return T;
    }

    double DistanceToSO3(const Matrix3d &M) {
        if (M.determinant() > 0)
            return (M.transpose() * M - Matrix3d::Identity()).norm();
        else
            return 1.0e9;
    }

    double DistanceToSE3(const Matrix4d &T) {
        Matrix3d matR = T.block<3, 3>(0, 0);
        if (matR.determinant() > 0) {
            Matrix4d m_ret;
            m_ret << matR.transpose() * matR, Vector3d::Zero(3),
                    T.row(3);
            m_ret = m_ret - Matrix4d::Identity();
            return m_ret.norm();
        } else
            return 1.0e9;
    }

    bool TestIfSO3(const Matrix3d &M) {
        return std::abs(DistanceToSO3(M)) < 1e-3;
    }

    bool TestIfSE3(const Matrix4d &T) {
        return std::abs(DistanceToSE3(T)) < 1e-3;
    }
//
//    bool IKinBody(const MatrixXd &Blist, const MatrixXd &M, const MatrixXd &Tdes, VectorXd &thetalist, const MatrixXd& joint_limits,
//                        const Eigen::VectorXd& coupling_coefficients, double eomg, double ev) {
//        int i = 0;
//        int maxiterations = 20;
//
//        MatrixXd Tfk = FKinBody(M, Blist, thetalist);
//        MatrixXd Tdiff = TransInv(Tfk) * Tdes;
//        Tdiff.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
//
//        VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
//        Vector3d angular(Vb(0), Vb(1), Vb(2));
//        Vector3d linear(Vb(3), Vb(4), Vb(5));
//        bool err = linear.norm() > ev;
//        MatrixXd Jb;
//
//        cout << thetalist.size() << endl;
//
//        while (err && i < maxiterations) {
//
//            Jb = JacobianBody(Blist, thetalist);
//
//            cout << Jb.rows() << " , " << Jb.cols() << endl;
//            MatrixXd Jb_pinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
//
//            // use the extended Jacobian method to couple the DIP and PIP joints and
//            // set joint limits
//            MatrixXd null_space_projector = MatrixXd::Identity(thetalist.size(), thetalist.size()) - Jb_pinv * Jb;
//
//            // Compute the additional constraint rows
//            Eigen::MatrixXd additional_rows(thetalist.size(), thetalist.size());
//            for (int i = 0; i < thetalist.size(); ++i) {
//                additional_rows(i, i) = 0.0;
//                if (thetalist(i) < joint_limits(i, 0)) {
//                    additional_rows(i, i) = 1.0;
//                }
//                else if (thetalist(i) > joint_limits(i, 1)) {
//                    additional_rows(i, i) = -1.0;
//                }
//                if (i > 0 && coupling_coefficients(i) != 0.0) {
//                    additional_rows(i, i-1) = -coupling_coefficients(i);
//                    additional_rows(i, i) += coupling_coefficients(i);
//                }
//            }
//
//            // Compute the extended Jacobian matrix
//            MatrixXd J_extended = Jb;
//            J_extended.conservativeResize(Jb.rows() + additional_rows.rows(), Jb.cols());
//            J_extended.bottomRows(additional_rows.rows()) = additional_rows;
//            cout << additional_rows.rows() << endl;
//            cout << J_extended.rows() << " , " << J_extended.cols() << endl;
//            cout << null_space_projector.rows() << " , " << null_space_projector.cols() << endl;
//            J_extended = null_space_projector * J_extended;
//
//            cout << J_extended.rows() << " , " << J_extended.cols() << endl;
//
//            VectorXd d_thetalist = J_extended.completeOrthogonalDecomposition().pseudoInverse()*Vb ;
//            thetalist += d_thetalist;
//            i += 1;
//
//            // iterate
//            Tfk = FKinBody(M, Blist, thetalist);
//            Tdiff = TransInv(Tfk) * Tdes;
//            Tdiff.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
//            Vb = se3ToVec(MatrixLog6(Tdiff));
//            angular = Vector3d(Vb(0), Vb(1), Vb(2));
//            linear = Vector3d(Vb(3), Vb(4), Vb(5));
//            err = linear.norm() > ev;
//        }
//        cout << i << endl;
//
//        return !err;
//    }


//    bool IKinBody(const MatrixXd &Blist, const MatrixXd &M, const MatrixXd &Tdes, VectorXd &thetalist,
//                  const MatrixXd& joint_limits, const Eigen::VectorXd& coupling_coefficients,
//                  double eomg, double ev) {
//        int i = 0;
//        int maxiterations = 20;
//
//        MatrixXd Tfk = FKinBody(M, Blist, thetalist);
//        MatrixXd Tdiff = TransInv(Tfk) * Tdes;
//        Tdiff.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
//
//        VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
//        Vector3d angular(Vb(0), Vb(1), Vb(2));
//        Vector3d linear(Vb(3), Vb(4), Vb(5));
//        bool err = linear.norm() > ev;
//        MatrixXd Jb;
//
//        cout << thetalist.size() << endl;
//
//        while (err && i < maxiterations) {
//
//            Jb = JacobianBody(Blist, thetalist);
//
//            cout << Jb.rows() << " , " << Jb.cols() << endl;
//            MatrixXd Jb_pinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
//
//            // use the extended Jacobian method to couple the DIP and PIP joints and
//            // set joint limits
//            MatrixXd null_space_projector = MatrixXd::Identity(thetalist.size(), thetalist.size()) - Jb_pinv * Jb;
//
//            // Compute the additional constraint rows
//            Eigen::MatrixXd additional_rows(thetalist.size(), thetalist.size());
//            for (int i = 0; i < thetalist.size(); ++i) {
//                additional_rows(i, i) = 0.0;
//                if (thetalist(i) < joint_limits(i, 0)) {
//                    additional_rows(i, i) = 1.0;
//                }
//                else if (thetalist(i) > joint_limits(i, 1)) {
//                    additional_rows(i, i) = -1.0;
//                }
//                if (i > 0 && coupling_coefficients(i) != 0.0) {
//                    additional_rows(i, i-1) = -coupling_coefficients(i);
//                    additional_rows(i, i) += coupling_coefficients(i);
//                }
//            }
//
//            // Compute the extended Jacobian matrix
//            MatrixXd J_extended = Jb;
//            J_extended.conservativeResize(Jb.rows() + additional_rows.rows(), Jb.cols());
//            J_extended.bottomRows(additional_rows.rows()) = additional_rows;
//            cout << additional_rows.rows() << endl;
//            cout << J_extended.rows() << " , " << J_extended.cols() << endl;
//            cout << null_space_projector.rows() << " , " << null_space_projector.cols() << endl;
//            J_extended = null_space_projector * J_extended;
//
//            cout << J_extended.rows() << " , " << J_extended.cols() << endl;
//
//            VectorXd d_thetalist = J_extended.completeOrthogonalDecomposition().pseudoInverse()*Vb ;
//            thetalist += d_thetalist;
//            i += 1;
//
//            // iterate
//            Tfk = FKinBody(M, Blist, thetalist);
//            Tdiff = TransInv(Tfk) * Tdes;
//            Tdiff.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
//            Vb = se3ToVec(MatrixLog6(Tdiff));
//            angular = Vector3d(Vb(0), Vb(1), Vb(2));
//            linear = Vector3d(Vb(3), Vb(4), Vb(5));
//            err = linear.norm() > ev;
//        }
//        cout << i << endl;
//
//        return !err;
//    }

    bool IKinBodyDigit(const MatrixXd &Blist, const MatrixXd &M, const MatrixXd &Tdes, VectorXd &thetalist, double ev,
                       const MatrixXd& joint_limits, double k_null_limits,
                       double coupling_coefficient, double k_null_coupling) {
        // Initialize
        int i = 0;
        int maxiterations = 20;
        bool err = true;

        while (err && i < maxiterations){

            // Primary Objective: Position
            MatrixXd Tfk = FKinBody(M, Blist, thetalist);
            MatrixXd Tdiff = TransInv(Tfk) * Tdes;
            VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
            Vector3d vb(Vb(3), Vb(4), Vb(5));

            // Secondary Objective: Joint Limits
            Vector4d q_dot_null_limits;
            for (int i = 0; i < thetalist.size(); ++i) {
                q_dot_null_limits(i) = -(thetalist(i)-joint_limits(i,0))/(joint_limits(i,2)-joint_limits(i,1));
            }

            // Tertiary Objective: Joint Coupling
            Vector4d q_dot_null_coupling;
            q_dot_null_coupling(0) = 0;
            q_dot_null_coupling(1) = 0;
            q_dot_null_coupling(2) = -( thetalist(3)/thetalist(2) - coupling_coefficient )*( thetalist(3)/pow(thetalist(2),2));
            q_dot_null_coupling(3) = -( thetalist(3)/thetalist(2) - coupling_coefficient )/thetalist(2);

            // Linearization
            MatrixXd Jb = JacobianBody(Blist, thetalist);


            MatrixXd Jb_v = Jb.bottomRows<3>();

            // Moore-Penrose inverse
            MatrixXd J_dagger = Jb_v.completeOrthogonalDecomposition().pseudoInverse();

//            cout << J_dagger << endl;

            // Redundancy Resolution
            MatrixXd null_space_projector = MatrixXd::Identity(thetalist.size(), thetalist.size()) - J_dagger * Jb_v;

            cout << "NULL SPACE" << endl;
            cout << null_space_projector << endl;

            VectorXd d_thetalist = J_dagger*vb;
//                                 + null_space_projector*(k_null_limits*q_dot_null_limits + k_null_coupling*q_dot_null_coupling);

            cout << "PROJECTED COST" <<endl;
            cout << null_space_projector*(k_null_limits*q_dot_null_limits + k_null_coupling*q_dot_null_coupling) << endl;
//            cout << "theta" << endl;
//            cout << d_thetalist.transpose() << endl;
//
//            cout << " J "<< endl;
//            cout << J_dagger << endl;

            // Update
            thetalist += d_thetalist;
            i += 1;
            err = vb.norm() > ev;

        }

        return !err;
    }


    bool IKinSpaceDigit(const MatrixXd &Slist, const MatrixXd &M, const MatrixXd &Tdes, VectorXd &thetalist, double ev,
                   const MatrixXd& joint_limits, double k_null_limits,
                   double coupling_coefficient, double k_null_coupling) {

        // Initialize
        int i = 0;
        int maxiterations = 20;
        bool err = true;

        while (err && i < maxiterations) {

            std::cout << " IK " << std::endl;
            // Primary Objective: Position
            MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
            MatrixXd Tdiff = TransInv(Tfk) * Tdes;
            VectorXd Vs = Adjoint(Tfk) * se3ToVec(MatrixLog6(Tdiff));
            Vector3d vs(Vs(3), Vs(4), Vs(5));

            // Secondary Objective: Joint Limits
            Vector4d q_dot_null_limits;
            for (int i = 0; i < thetalist.size(); ++i) {
                q_dot_null_limits(i) = -(thetalist(i)-joint_limits(i,0))/(joint_limits(i,2)-joint_limits(i,1));
            }

            // Tertiary Objective: Joint Coupling
            Vector4d q_dot_null_coupling;
            q_dot_null_coupling(0) = 0;
            q_dot_null_coupling(1) = 0;
            q_dot_null_coupling(2) = -( thetalist(3)/thetalist(2) - coupling_coefficient )*( thetalist(3)/pow(thetalist(2),2));
            q_dot_null_coupling(3) = -( thetalist(3)/thetalist(2) - coupling_coefficient )/thetalist(2);

            // Linearization
            MatrixXd Js = JacobianSpace(Slist, thetalist);
            MatrixXd Js_v = Js.bottomRows<3>();

            // Moore-Penrose Inverse
            MatrixXd Js_dagger = Js_v.completeOrthogonalDecomposition().pseudoInverse();
            // Redundancy Resolution
            MatrixXd null_space_projector = MatrixXd::Identity(thetalist.size(), thetalist.size()) - Js_dagger * Js_v;

            VectorXd d_thetalist = Js_dagger*vs
                                 + null_space_projector*(k_null_limits*q_dot_null_limits + k_null_coupling*q_dot_null_coupling);

            // Update
            i += 1;
            thetalist += d_thetalist;
            err = vs.norm() > ev;
        }

        return !err;
    }

}