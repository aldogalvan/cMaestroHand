#define OPTIM_ENABLE_EIGEN_WRAPPERS
//#include "optim.hpp"
#include "mr.h"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
//#include "src/stdafx.h"
//#include "src/interpolation.h"

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
    Matrix3d VecToso3(const Vector3d& omg) {
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
    Vector3d so3ToVec(const MatrixXd& so3mat) {
        Vector3d v_ret;
        v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return v_ret;
    }


    /* Function: Translates an exponential rotation into it's individual components
     * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
     *				and the angle of rotation)
     * Returns: The axis and angle of rotation as [x, y, z, theta]
     */
    Vector4d AxisAng3(const Vector3d& expc3) {
        Vector4d v_ret;
        v_ret << Normalize(expc3), expc3.norm();
        return v_ret;
    }


    /* Function: Translates an exponential rotation into a rotation matrix
     * Inputs: exponenential representation of a rotation
     * Returns: Rotation matrix
     */
    Matrix3d MatrixExp3(const Matrix3d& so3mat) {
        Vector3d omgtheta = so3ToVec(so3mat);

        Matrix3d m_ret = Matrix3d::Identity();
        if (NearZero(so3mat.norm())) {
            return m_ret;
        }
        else {
            double theta = (AxisAng3(omgtheta))(3);
            Matrix3d omgmat = so3mat * (1 / theta);
            return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
        }
    }


    /* Function: Computes the matrix logarithm of a rotation matrix
     * Inputs: Rotation matrix
     * Returns: matrix logarithm of a rotation
     */
    Matrix3d MatrixLog3(const Matrix3d& R) {
        double acosinput = (R.trace() - 1) / 2.0;
        MatrixXd m_ret = MatrixXd::Zero(3, 3);
        if (acosinput >= 1)
            return m_ret;
        else if (acosinput <= -1) {
            Vector3d omg;
            if (!NearZero(1 + R(2, 2)))
                omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
            else if (!NearZero(1 + R(1, 1)))
                omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
            else
                omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
            m_ret = VecToso3(M_PI * omg);
            return m_ret;
        }
        else {
            double theta = std::acos(acosinput);
            m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
            return m_ret;
        }
    }

    /* Function: Combines a rotation matrix and position vector into a single
     * 				Special Euclidian Group (SE3) homogeneous transformation matrix
     * Inputs: Rotation Matrix (R), Position Vector (p)
     * Returns: Matrix of T = [ [R, p],
     *						    [0, 1] ]
     */
    MatrixXd RpToTrans(const Matrix3d& R, const Vector3d& p) {
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
    std::vector<MatrixXd> TransToRp(const MatrixXd& T) {
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
    MatrixXd VecTose3(const VectorXd& V) {
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
    VectorXd se3ToVec(const MatrixXd& T) {
        VectorXd m_ret(6);
        m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

        return m_ret;
    }


    /* Function: Provides the adjoint representation of a transformation matrix
     *			 Used to change the frame of reference for spatial velocity vectors
     * Inputs: 4x4 Transformation matrix SE(3)
     * Returns: 6x6 Adjoint Representation of the matrix
     */
    MatrixXd Adjoint(const MatrixXd& T) {
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
    MatrixXd MatrixExp6(const MatrixXd& se3mat) {
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
            Matrix3d expExpand = MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
            Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
            Vector3d GThetaV = (expExpand*linear) / theta;
            m_ret << MatrixExp3(se3mat_cut), GThetaV,
                    0, 0, 0, 1;
            return m_ret;
        }

    }

    MatrixXd MatrixLog6(const MatrixXd& T) {
        MatrixXd m_ret(4, 4);
        auto rp = mr::TransToRp(T);
        Matrix3d omgmat = MatrixLog3(rp.at(0));
        Matrix3d zeros3d = Matrix3d::Zero(3, 3);
        if (NearZero(omgmat.norm())) {
            m_ret << zeros3d, rp.at(1),
                    0, 0, 0, 0;
        }
        else {
            double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
            Matrix3d logExpand1 = MatrixXd::Identity(3, 3) - omgmat / 2.0;
            Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
            Matrix3d logExpand = logExpand1 + logExpand2;
            m_ret << omgmat, logExpand*rp.at(1),
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
    MatrixXd FKinSpace(const MatrixXd& M, const MatrixXd& Slist, const VectorXd& thetaList) {
        MatrixXd T = M;
        for (int i = (thetaList.size() - 1); i > -1; i--) {
            T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
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
    MatrixXd FKinBody(const MatrixXd& M, const MatrixXd& Blist, const VectorXd& thetaList) {
        MatrixXd T = M;
        for (int i = 0; i < thetaList.size(); i++) {
            T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
        }
        return T;
    }


    /* Function: Gives the space Jacobian
     * Inputs: Screw axis in home position, joint configuration
     * Returns: 6xn Spatial Jacobian
     */
    MatrixXd JacobianSpace(const MatrixXd& Slist, const MatrixXd& thetaList) {
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
    MatrixXd JacobianBody(const MatrixXd& Blist, const MatrixXd& thetaList) {
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

    MatrixXd TransInv(const MatrixXd& transform) {
        auto rp = mr::TransToRp(transform);
        auto Rt = rp.at(0).transpose();
        auto t = -(Rt * rp.at(1));
        MatrixXd inv(4, 4);
        inv = MatrixXd::Zero(4,4);
        inv.block(0, 0, 3, 3) = Rt;
        inv.block(0, 3, 3, 1) = t;
        inv(3, 3) = 1;
        return inv;
    }

    MatrixXd RotInv(const MatrixXd& rotMatrix) {
        return rotMatrix.transpose();
    }

    VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h) {
        VectorXd axis(6);
        axis.segment(0, 3) = s;
        axis.segment(3, 3) = q.cross(s) + (h * s);
        return axis;
    }

    VectorXd AxisAng6(const VectorXd& expc6) {
        VectorXd v_ret(7);
        double theta = Vector3d(expc6(0), expc6(1), expc6(2)).norm();
        if (NearZero(theta))
            theta = Vector3d(expc6(3), expc6(4), expc6(5)).norm();
        v_ret << expc6 / theta, theta;
        return v_ret;
    }

    MatrixXd ProjectToSO3(const MatrixXd& M) {
        JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullV);
        MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0)
            // In this case the result may be far from M; reverse sign of 3rd column
            R.col(2) *= -1;
        return R;
    }

    MatrixXd ProjectToSE3(const MatrixXd& M) {
        Matrix3d R = M.block<3, 3>(0, 0);
        Vector3d t = M.block<3, 1>(0, 3);
        MatrixXd T = RpToTrans(ProjectToSO3(R), t);
        return T;
    }

    double DistanceToSO3(const Matrix3d& M) {
        if (M.determinant() > 0)
            return (M.transpose() * M - Matrix3d::Identity()).norm();
        else
            return 1.0e9;
    }

    double DistanceToSE3(const Matrix4d& T) {
        Matrix3d matR = T.block<3, 3>(0, 0);
        if (matR.determinant() > 0) {
            Matrix4d m_ret;
            m_ret << matR.transpose()*matR, Vector3d::Zero(3),
                    T.row(3);
            m_ret = m_ret - Matrix4d::Identity();
            return m_ret.norm();
        }
        else
            return 1.0e9;
    }

    bool TestIfSO3(const Matrix3d& M) {
        return std::abs(DistanceToSO3(M)) < 1e-3;
    }

    bool TestIfSE3(const Matrix4d& T) {
        return std::abs(DistanceToSE3(T)) < 1e-3;
    }

    // TODO: Constrain the DIP angle to the PIP
    bool IKinBody(const MatrixXd& Blist, const MatrixXd& M, const MatrixXd& Tdes, const MatrixXd& Tstart, MatrixXd& Tend,
                  VectorXd& thetalist, const VectorXd& thetaopt, const VectorXd& lb, const VectorXd& ub, double eomg, double ev) {
        int i = 0;
        int maxiterations = 20;
        int sz = thetalist.size();

        MatrixXd Tfk = FKinBody(M, Blist, thetalist);
        MatrixXd Tdiff = TransInv(Tfk)*Tdes;
        Tdiff.block<3,3>(0,0) = MatrixXd::Identity(3,3);

        VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
        Vector3d angular(Vb(0), Vb(1), Vb(2));
        Vector3d linear(Vb(3), Vb(4), Vb(5));
        bool err =  linear.norm() > ev;
        MatrixXd Jb;
        while (err && i < maxiterations) {

            Jb = JacobianBody(Blist, thetalist);
            MatrixXd Jb_pinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
            MatrixXd primary_goal = Jb_pinv*Vb;

            // check if any constraints are violated
            for (int i = 0; i < 4; i++)
            {
                if (thetalist[i] < lb[i])
                {
                    // set a penalty returning the constraint
                    thetalist[i] = lb[i];
                    // saturate the jacobian
                    if (primary_goal(i) < 0 )
                    {
                        primary_goal(i) *= -1;
                    }

                }

                if (thetalist[i] > ub[i])
                {
                    thetalist[i] = ub[i];
                    if (primary_goal(i) > 0 )
                    {
                        primary_goal(i) *= -1;
                    }
                }
            }


            VectorXd d_thetalist =  primary_goal + (MatrixXd::Identity(4,4) - Jb_pinv*Jb)*(thetaopt - thetalist);
            thetalist += d_thetalist;
            i += 1;

            // iterate
            Tfk = FKinBody(M, Blist, thetalist);
            Tdiff = TransInv(Tfk)*Tdes;
            Tdiff.block<3,3>(0,0) = MatrixXd::Identity(3,3);
            Vb = se3ToVec(MatrixLog6(Tdiff));
            angular = Vector3d(Vb(0), Vb(1), Vb(2));
            linear = Vector3d(Vb(3), Vb(4), Vb(5));
            err =  linear.norm() > ev;
        }
        cout << i << endl;
        return !err;
    }

    bool IKinSpace(const MatrixXd& Slist, const MatrixXd& M, const MatrixXd& Tdes, const MatrixXd& Tstart, MatrixXd& Tend,
                   VectorXd& thetalist, const VectorXd& thetaopt, const VectorXd& lb, const VectorXd& ub, double eomg, double ev) {
        int i = 0;
        int maxiterations = 20;
        int sz = thetalist.size();

        // MatrixXd T = TransInv(Tstart)*Tdes;

        for (int i = 0; i  < sz ; i ++)
        {
            if (thetalist[i] < lb[i])
                thetalist[i] = lb[i];
            else if ( thetalist[i] > ub[i])
                thetalist[i] = ub[i];
        }

        MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
        // Tfk = TransInv(Tstart)*Tfk;
        MatrixXd Tdiff = TransInv(Tfk)*Tdes;
        VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
        Vector3d angular(Vs(0), Vs(1), Vs(2));
        Vector3d linear(Vs(3), Vs(4), Vs(5));
        bool err = linear.norm() > ev;
        MatrixXd Js;

        // TODO: implement joint limits
        while (err && i < maxiterations) {
            Js = JacobianSpace(Slist, thetalist);
            thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
            //MatrixXd Js_pinv = Js.completeOrthogonalDecomposition().pseudoInverse();
            //VectorXd d_thetalist = Js_pinv*Vs + (MatrixXd::Identity(4,4) - Js_pinv*Js)*(thetaopt - thetalist);
            //thetalist += d_thetalist;

            // set the joint limits
            for (int i = 0; i  < sz ; i ++)
            {
                if (thetalist[i] < lb[i]) {
                    //cout << "ub" << endl;
                    thetalist[i] = lb[i];
                }
                else if ( thetalist[i] > ub[i]) {
                    //cout << "lb" << endl;
                    thetalist[i] = ub[i];
                }
            }
            i += 1;
            // iterate
            Tfk = FKinSpace(M, Slist, thetalist);
            //Tfk = TransInv(Tstart)*Tfk;
            Tdiff = TransInv(Tfk)*Tdes;
            Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
            angular = Vector3d(Vs(0), Vs(1), Vs(2));
            linear = Vector3d(Vs(3), Vs(4), Vs(5));
            err = linear.norm() > ev;
        }
        Tend = Tfk;

        return !err;
    }

    /*
    * Function: This function uses forward-backward Newton-Euler iterations to solve the
    * equation:
    * taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
    *           + g(thetalist) + Jtr(thetalist) * Ftip
    * Inputs:
    *  thetalist: n-vector of joint variables
    *  dthetalist: n-vector of joint rates
    *  ddthetalist: n-vector of joint accelerations
    *  g: Gravity vector g
    *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
    *  Mlist: List of link frames {i} relative to {i-1} at the home position
    *  Glist: Spatial inertia matrices Gi of the links
    *  Slist: Screw axes Si of the joints in a space frame, in the format
    *         of a matrix with the screw axes as the columns.
    *
    * Outputs:
    *  taulist: The n-vector of required joint forces/torques
    *
    */

    VectorXd InverseDynamics(const VectorXd& thetalist, const VectorXd& dthetalist, const VectorXd& ddthetalist,
                                    const VectorXd& g, const VectorXd& Ftip, const std::vector<MatrixXd>& Mlist,
                                    const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {
        // the size of the lists
        int n = thetalist.size();

        MatrixXd Mi = MatrixXd::Identity(4, 4);
        MatrixXd Ai = MatrixXd::Zero(6,n);
        std::vector<MatrixXd> AdTi;
        for (int i = 0; i < n+1; i++) {
            AdTi.push_back(MatrixXd::Zero(6,6));
        }
        MatrixXd Vi = MatrixXd::Zero(6,n+1);    // velocity
        MatrixXd Vdi = MatrixXd::Zero(6,n+1);   // acceleration

        Vdi.block(3, 0, 3, 1) = - g;
        AdTi[n] = mr::Adjoint(mr::TransInv(Mlist[n]));
        VectorXd Fi = Ftip;

        VectorXd taulist = VectorXd::Zero(n);

        // forward pass
        for (int i = 0; i < n; i++) {
            Mi = Mi * Mlist[i];
            Ai.col(i) = mr::Adjoint(mr::TransInv(Mi))*Slist.col(i);

            AdTi[i] = mr::Adjoint(mr::MatrixExp6(mr::VecTose3(Ai.col(i)*-thetalist(i)))
                                  * mr::TransInv(Mlist[i]));

            Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
            Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
                           + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
        }

        // backward pass
        for (int i = n-1; i >= 0; i--) {
            Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
                 - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
            taulist(i) = Fi.transpose() * Ai.col(i);
        }
        return taulist;
    }

    /*
     * Function: This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
     *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  g: Gravity vector g
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  grav: The 3-vector showing the effect force of gravity to the dynamics
     *
     */
    VectorXd GravityForces(const VectorXd& thetalist, const VectorXd& g,
                                  const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {
        int n = thetalist.size();
        VectorXd dummylist = VectorXd::Zero(n);
        VectorXd dummyForce = VectorXd::Zero(6);
        VectorXd grav = mr::InverseDynamics(thetalist, dummylist, dummylist, g,
                                                   dummyForce, Mlist, Glist, Slist);
        return grav;
    }

    /*
       * Function: This function calls InverseDynamics n times, each time passing a
     * ddthetalist vector with a single element equal to one and all other
     * inputs set to zero. Each call of InverseDynamics generates a single
     * column, and these columns are assembled to create the inertia matrix.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
     *     chain at the given configuration thetalist.
     */
    MatrixXd MassMatrix(const VectorXd& thetalist,
                               const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {
        int n = thetalist.size();
        VectorXd dummylist = VectorXd::Zero(n);
        VectorXd dummyg = VectorXd::Zero(3);
        VectorXd dummyforce = VectorXd::Zero(6);
        MatrixXd M = MatrixXd::Zero(n,n);
        for (int i = 0; i < n; i++) {
            VectorXd ddthetalist = VectorXd::Zero(n);
            ddthetalist(i) = 1;
            M.col(i) = mr::InverseDynamics(thetalist, dummylist, ddthetalist,
                                           dummyg, dummyforce, Mlist, Glist, Slist);
        }
        return M;
    }

    /*
       * Function: This function calls InverseDynamics with g = 0, Ftip = 0, and
     * ddthetalist = 0.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: A list of joint rates
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
     *     terms for a given thetalist and dthetalist.
     */
    VectorXd VelQuadraticForces(const VectorXd& thetalist, const VectorXd& dthetalist,
                                       const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {
        int n = thetalist.size();
        VectorXd dummylist = VectorXd::Zero(n);
        VectorXd dummyg = VectorXd::Zero(3);
        VectorXd dummyforce = VectorXd::Zero(6);
        VectorXd c = mr::InverseDynamics(thetalist, dthetalist, dummylist,
                                                dummyg, dummyforce, Mlist, Glist, Slist);
        return c;
    }

    /*
       * Function: This function calls InverseDynamics with g = 0, dthetalist = 0, and
     * ddthetalist = 0.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  JTFtip: The joint forces and torques required only to create the
     *     end-effector force Ftip.
     */
    VectorXd EndEffectorForces(const VectorXd& thetalist, const VectorXd& Ftip,
                                      const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {
        int n = thetalist.size();
        VectorXd dummylist = VectorXd::Zero(n);
        VectorXd dummyg = VectorXd::Zero(3);

        VectorXd JTFtip = mr::InverseDynamics(thetalist, dummylist, dummylist,
                                                     dummyg, Ftip, Mlist, Glist, Slist);
        return JTFtip;
    }

    /*
     * Function: This function computes ddthetalist by solving:
     * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist)
     *                                  - g(thetalist) - Jtr(thetalist) * Ftip
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: n-vector of joint rates
     *  taulist: An n-vector of joint forces/torques
     *  g: Gravity vector g
     *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  ddthetalist: The resulting joint accelerations
     *
     */
    VectorXd ForwardDynamics(const VectorXd& thetalist, const VectorXd& dthetalist, const VectorXd& taulist,
                                    const VectorXd& g, const VectorXd& Ftip, const std::vector<MatrixXd>& Mlist,
                                    const std::vector<MatrixXd>& Glist, const MatrixXd& Slist) {

        VectorXd totalForce = taulist - mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
                                     - mr::GravityForces(thetalist, g, Mlist, Glist, Slist)
                                     - mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

        MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

        // Use LDLT since M is positive definite
        VectorXd ddthetalist = M.ldlt().solve(totalForce);

        return ddthetalist;
    }

    void EulerStep(VectorXd& thetalist, VectorXd& dthetalist, const VectorXd& ddthetalist, double dt) {
        thetalist += dthetalist * dt;
        dthetalist += ddthetalist * dt;
        return;
    }

    MatrixXd InverseDynamicsTrajectory(const MatrixXd& thetamat, const MatrixXd& dthetamat, const MatrixXd& ddthetamat,
                                              const VectorXd& g, const MatrixXd& Ftipmat, const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist,
                                              const MatrixXd& Slist) {
        MatrixXd thetamatT = thetamat.transpose();
        MatrixXd dthetamatT = dthetamat.transpose();
        MatrixXd ddthetamatT = ddthetamat.transpose();
        MatrixXd FtipmatT = Ftipmat.transpose();

        int N = thetamat.rows();  // trajectory points
        int dof = thetamat.cols();
        MatrixXd taumatT = MatrixXd::Zero(dof, N);
        for (int i = 0; i < N; ++i) {
            taumatT.col(i) = InverseDynamics(thetamatT.col(i), dthetamatT.col(i), ddthetamatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
        }
        MatrixXd taumat = taumatT.transpose();
        return taumat;
    }

    std::vector<MatrixXd> ForwardDynamicsTrajectory(const VectorXd& thetalist, const VectorXd& dthetalist, const MatrixXd& taumat,
                                                           const VectorXd& g, const MatrixXd& Ftipmat, const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist,
                                                           const MatrixXd& Slist, double dt, int intRes) {
        MatrixXd taumatT = taumat.transpose();
        MatrixXd FtipmatT = Ftipmat.transpose();
        int N = taumat.rows();  // force/torque points
        int dof = taumat.cols();
        MatrixXd thetamatT = MatrixXd::Zero(dof, N);
        MatrixXd dthetamatT = MatrixXd::Zero(dof, N);
        thetamatT.col(0) = thetalist;
        dthetamatT.col(0) = dthetalist;
        VectorXd thetacurrent = thetalist;
        VectorXd dthetacurrent = dthetalist;
        VectorXd ddthetalist;
        for (int i = 0; i < N - 1; ++i) {
            for (int j = 0; j < intRes; ++j) {
                ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taumatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
                EulerStep(thetacurrent, dthetacurrent, ddthetalist, 1.0*dt / intRes);
            }
            thetamatT.col(i + 1) = thetacurrent;
            dthetamatT.col(i + 1) = dthetacurrent;
        }
        std::vector<MatrixXd> JointTraj_ret;
        JointTraj_ret.push_back(thetamatT.transpose());
        JointTraj_ret.push_back(dthetamatT.transpose());
        return JointTraj_ret;
    }

    VectorXd ComputedTorque(const VectorXd& thetalist, const VectorXd& dthetalist, const VectorXd& eint,
                                   const VectorXd& g, const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist,
                                   const MatrixXd& Slist, const VectorXd& thetalistd, const VectorXd& dthetalistd, const VectorXd& ddthetalistd,
                                   double Kp, double Ki, double Kd) {

        VectorXd e = thetalistd - thetalist;  // position err
        VectorXd tau_feedforward = MassMatrix(thetalist, Mlist, Glist, Slist)*(Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

        VectorXd Ftip = VectorXd::Zero(6);
        VectorXd tau_inversedyn = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

        VectorXd tau_computed = tau_feedforward + tau_inversedyn;
        return tau_computed;
    }

    double CubicTimeScaling(double Tf, double t) {
        double timeratio = 1.0*t / Tf;
        double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
        return st;
    }

    double QuinticTimeScaling(double Tf, double t) {
        double timeratio = 1.0*t / Tf;
        double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
        return st;
    }

    MatrixXd JointTrajectory(const VectorXd& thetastart, const VectorXd& thetaend, double Tf, int N, int method) {
        double timegap = Tf / (N - 1);
        MatrixXd trajT = MatrixXd::Zero(thetastart.size(), N);
        double st;
        for (int i = 0; i < N; ++i) {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap*i);
            else
                st = QuinticTimeScaling(Tf, timegap*i);
            trajT.col(i) = st * thetaend + (1 - st)*thetastart;
        }
        MatrixXd traj = trajT.transpose();
        return traj;
    }
    std::vector<MatrixXd> ScrewTrajectory(const MatrixXd& Xstart, const MatrixXd& Xend, double Tf, int N, int method) {
        double timegap = Tf / (N - 1);
        std::vector<MatrixXd> traj(N);
        double st;
        for (int i = 0; i < N; ++i) {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap*i);
            else
                st = QuinticTimeScaling(Tf, timegap*i);
            MatrixXd Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
            traj.at(i) = Xstart * MatrixExp6(Ttemp*st);
        }
        return traj;
    }

    std::vector<MatrixXd> CartesianTrajectory(const MatrixXd& Xstart, const MatrixXd& Xend, double Tf, int N, int method) {
        double timegap = Tf / (N - 1);
        std::vector<MatrixXd> traj(N);
        std::vector<MatrixXd> Rpstart = TransToRp(Xstart);
        std::vector<MatrixXd> Rpend = TransToRp(Xend);
        Matrix3d Rstart = Rpstart[0]; Vector3d pstart = Rpstart[1];
        Matrix3d Rend = Rpend[0]; Vector3d pend = Rpend[1];
        double st;
        for (int i = 0; i < N; ++i) {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap*i);
            else
                st = QuinticTimeScaling(Tf, timegap*i);
            Matrix3d Ri = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend)*st);
            Vector3d pi = st*pend + (1 - st)*pstart;
            MatrixXd traji(4, 4);
            traji << Ri, pi,
                    0, 0, 0, 1;
            traj.at(i) = traji;
        }
        return traj;
    }
    std::vector<MatrixXd> SimulateControl(const VectorXd& thetalist, const VectorXd& dthetalist, const VectorXd& g,
                                                 const MatrixXd& Ftipmat, const std::vector<MatrixXd>& Mlist, const std::vector<MatrixXd>& Glist,
                                                 const MatrixXd& Slist, const MatrixXd& thetamatd, const MatrixXd& dthetamatd, const MatrixXd& ddthetamatd,
                                                 const VectorXd& gtilde, const std::vector<MatrixXd>& Mtildelist, const std::vector<MatrixXd>& Gtildelist,
                                                 double Kp, double Ki, double Kd, double dt, int intRes) {
        MatrixXd FtipmatT = Ftipmat.transpose();
        MatrixXd thetamatdT = thetamatd.transpose();
        MatrixXd dthetamatdT = dthetamatd.transpose();
        MatrixXd ddthetamatdT = ddthetamatd.transpose();
        int m = thetamatdT.rows(); int n = thetamatdT.cols();
        VectorXd thetacurrent = thetalist;
        VectorXd dthetacurrent = dthetalist;
        VectorXd eint = VectorXd::Zero(m);
        MatrixXd taumatT = MatrixXd::Zero(m, n);
        MatrixXd thetamatT = MatrixXd::Zero(m, n);
        VectorXd taulist;
        VectorXd ddthetalist;
        for (int i = 0; i < n; ++i) {
            taulist = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, Mtildelist, Gtildelist, Slist, thetamatdT.col(i),
                                     dthetamatdT.col(i), ddthetamatdT.col(i), Kp, Ki, Kd);
            for (int j = 0; j < intRes; ++j) {
                ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, FtipmatT.col(i), Mlist, Glist, Slist);
                EulerStep(thetacurrent, dthetacurrent, ddthetalist, dt / intRes);
            }
            taumatT.col(i) = taulist;
            thetamatT.col(i) = thetacurrent;
            eint += dt * (thetamatdT.col(i) - thetacurrent);
        }
        std::vector<MatrixXd> ControlTauTraj_ret;
        ControlTauTraj_ret.push_back(taumatT.transpose());
        ControlTauTraj_ret.push_back(thetamatT.transpose());
        return ControlTauTraj_ret;
    }
}