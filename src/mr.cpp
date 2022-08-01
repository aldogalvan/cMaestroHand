#include "mr.h"
#include <stdio.h>
#include <iostream>

double pi = 3.1412;

// -----------------------------------------------------------------//

 MatrixXd Adjoint(Matrix4d T)
{
    auto [R,p] = TransToRp(T);
    MatrixXd ret(6,6);
    ret.block<3,3>(0,0) = R;
    ret.block<3,3>(0,3) = Eigen::Matrix3d().setZero();
    ret.block<3,3>(3,0) = VecToso3(p) * R;
    ret.block<3,3>(3,3) = R;

    return ret;
}

// -----------------------------------------------------------------//

 tuple<Vector3d , double> AxisAng3(Vector3d expc3)
{
    auto theta = expc3.norm();
    auto omghat = expc3 / theta;

    return tuple<Vector3d,double>(omghat,theta);
}

// -----------------------------------------------------------------//

 tuple<VectorXd , double> AxisAng6(VectorXd expc6)
{
    auto theta = expc6.head(3).norm();
    if (NearZero(theta))
        theta = expc6.tail(3).norm();
    auto S = expc6 / theta;

    return tuple<VectorXd, double>(S,theta);
}

// -----------------------------------------------------------------//

 double DistanceToSE3(Matrix4d mat)
{
    auto [R, p] = TransToRp(mat);
    if (R.determinant() > 0) {
        Matrix4d temp;
        temp.block<3, 3>(0, 0) = R.transpose() * R;
        temp.block<3, 1>(0, 3) = Vector3d().setZero();
        temp.block<1, 4>(3, 0) = mat.block<1, 4>(3, 0);
        return (temp - Matrix4d().setIdentity()).norm();
    } else {
        return 1e9;
    }
}

// -----------------------------------------------------------------//

 double DistanceToSO3(Matrix3d mat)
{
    if (mat.determinant() > 0)
    {
        return (mat.transpose()*mat - Matrix3d().setIdentity()).norm();
    }
    else
    {
        return 1e9;
    }
}

// -----------------------------------------------------------------//

 Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist)
{
    auto T = M;
    for (int i = 0u ; i  < thetalist.size(); i++)
    {
        T = T * MatrixExp6(VecTose3(Blist.block<6,1>(0,i)*thetalist(i)));
    }
    return T;
}

// -----------------------------------------------------------------//

 Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist)
{
    auto T = M;
    for (int i = 0u ; i  < thetalist.size(); i++)
    {
        T = MatrixExp6(VecTose3(Slist.block<6,1>(0,i)*thetalist(i))) * T;
    }
    return T;
}

// -----------------------------------------------------------------//

 bool IKinBody(MatrixXd Blist, Matrix4d M, Matrix4d T, Matrix4d& Tsb, const VectorXd thetalist0,
               VectorXd & thetalist, double eomg , double ev, const int maxIterations)
{
    thetalist = thetalist0;
    int i = 0u;
    auto Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M,Blist,thetalist))*T));
    auto err = Vb.head(3).norm() > eomg || Vb.tail(3).norm() > ev;
    while (err && i < maxIterations)
    {
        thetalist = thetalist + (JacobianBody(Blist,thetalist)*Vb).completeOrthogonalDecomposition().pseudoInverse();
        i++;
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M,Blist,thetalist))*T));
        err = Vb.head(3).norm() > eomg || Vb.tail(3).norm() > ev;
    }
    return !err;
}

// -----------------------------------------------------------------//

 bool IKinSpace(MatrixXd Slist, Matrix4d M, Matrix4d T, Matrix4d& Tsb, const VectorXd thetalist0,
                VectorXd & thetalist, double eomg , double ev, const int maxIterations)
{
    thetalist = thetalist0;
    int i = 0u;
    Tsb = FKinSpace(M,Slist,thetalist);
    VectorXd Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb)*T));
    auto err = Vs.head(3).norm() > eomg || Vs.tail(3).norm() > ev;
    while (err && i < maxIterations)
    {
        thetalist = thetalist + (JacobianBody(Slist,thetalist)*Vs);
        i++;
        Tsb = FKinSpace(M,Slist,thetalist);
        Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb)*T));
        err = Vs.head(3).norm() > eomg || Vs.tail(3).norm() > ev;
    }
    return !err;
}

// -----------------------------------------------------------------//

 MatrixXd JacobianBody(MatrixXd Blist, VectorXd thetalist)
{
    auto Jb = Blist;
    auto T = Matrix4d().setIdentity();
    for (int i = thetalist.size(); i > 0 ; i--)
    {
        T = T * MatrixExp6(VecTose3(-1*Blist.block<6,1>(0,i + 1)*thetalist(i + 1)));
        Jb.block<6,1>(0,i) = Adjoint(T) * Blist.block<6,1>(0,i);
    }
    return Jb;
}

// -----------------------------------------------------------------//

 MatrixXd JacobianSpace(MatrixXd Slist, VectorXd thetalist)
{
    auto Js = Slist;
    auto T = Matrix4d().setIdentity();
    for (int i = 1; i < thetalist.size() ; i++)
    {
        T = T * MatrixExp6(VecTose3(Slist.block<6,1>(0,i - 1)*thetalist(i - 1)));
        Js.block<6,1>(0,i) = Adjoint(T) * Slist.block<6,1>(0,i);
    }
    return Js;
}

// -----------------------------------------------------------------//

 Matrix3d MatrixExp3(Matrix3d so3mat)
{
    auto omgtheta = so3ToVec(so3mat);
    if (NearZero(omgtheta.norm()))
    {
        return Matrix3d().setIdentity();
    }
    else
    {
        auto [omghat,theta] = AxisAng3(omgtheta);
        auto omgmat = so3mat / theta;
        return Matrix3d().setIdentity() + sin(theta) * omgmat + (1 - cos(theta))*omgmat*omgmat;
    }
}

// -----------------------------------------------------------------//

 Matrix4d MatrixExp6(Matrix4d se3mat)
{
    auto omgtheta = so3ToVec(se3mat.block<3,3>(0,0));
    Matrix4d T;
    if (NearZero(omgtheta.norm()))
    {
        T.block<3,3>(0,0) = Matrix3d().setIdentity();
        T.block<3,1>(0,3) = se3mat.block<3,1>(0,3);
        T.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    }
    else
    {
        auto [omghat , theta] = AxisAng3(omgtheta);
        auto omgmat = se3mat.block<3,3>(0,0) / theta;
        T.block<3,3>(0,0) = MatrixExp3(se3mat.block<3,3>(0,0));
        T.block<3,1>(0,3) = (Matrix3d().setIdentity() * theta + (1-cos(theta))*omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat.block<3,1>(0,3) / theta;
        T.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    }
    return T;
}

// -----------------------------------------------------------------//

 Matrix3d MatrixLog3(Matrix3d R)
{
    auto acosinput = (R.trace() - 1) /2;
    Matrix3d so3mat;
    if (acosinput >= 1)
    {
        so3mat.setZero();
    }
    else if (acosinput <= -1)
    {
        Vector3d omg;
        if (!NearZero(1+R(2,2)))
        {
            omg = (1 / sqrt(2*(1+R(2,2))))*Vector3d(R(0,2),R(1,2),1+R(2,2));
        }
        else if (!NearZero(1+R(1,1)))
        {
            omg = (1 / sqrt(2*(1+R(1,1))))*Vector3d(R(0,1),1+R(1,1),R(2,1));
        }
        else
        {
            omg = (1 / sqrt(2*(1+R(0,0))))*Vector3d(1+R(0,0),R(1,0),R(2,0));
        }
        so3mat = VecToso3(pi * omg);
    }
    else
    {
        auto theta = acos(acosinput);
        so3mat = theta * (1/(2*sin(theta)))*(R - R.transpose());
    }
    return so3mat;
}

// -----------------------------------------------------------------//

 Matrix4d MatrixLog6(Matrix4d T)
{
    auto [R,p] = TransToRp(T);
    Matrix4d expmat;
    auto omgmat = MatrixLog3(R);
    if (omgmat.isApprox(Matrix3d().setZero()))
    {
        expmat.block<3,3>(0,0) = Matrix3d().setZero();
        expmat.block<3,1>(0,3) = T.block<3,1>(0,3);
        expmat.block<1,4>(3,0) << 0, 0, 0, 0;
    }
    else
    {
        auto theta = acos((R.trace() - 1) / 2);
        expmat.block<3,3>(0,0) = omgmat;
        expmat.block<3,1>(0,3) = (Matrix3d().setIdentity() - omgmat/2 + (1/theta - 1/(tan(theta/2)/2))*omgmat*omgmat/theta)*p;
        expmat.block<1,4>(3,0) << 0 , 0 , 0, 0;
    }
    return expmat;
}

// -----------------------------------------------------------------//

 bool NearZero(double near)
{
    if (abs(near) < 1e-6)
        return true;
    else
        return false;
}

// -----------------------------------------------------------------//

 Matrix4d ProjectToSE3(Matrix4d mat)
{
    return RpToTrans(ProjectToSO3(mat.block<3,3>(0,0)),mat.block<3,1>(0,3));
}

// -----------------------------------------------------------------//

 Matrix3d ProjectToSO3(Matrix3d mat)
{
    Eigen::JacobiSVD<MatrixXd> svd(mat,Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto U = svd.matrixU(); auto V = svd.matrixV();
    auto R = U * V.transpose();
    if (R.determinant() < 0)
    {
        // TODO: Check if this works
        -R.block<3, 1>(0, 2);
    }
    return R;
}

// -----------------------------------------------------------------//

 Matrix3d RotInv(Matrix3d R)
{
    return R.transpose();
}

// -----------------------------------------------------------------//

 Matrix4d RpToTrans(Matrix3d R, Vector3d p)
{
    Matrix4d T;
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    T.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    return T;
}

// -----------------------------------------------------------------//

 VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h)
{
    VectorXd S(6);
    S.head(3) = s;
    S.tail(3) = q.cross(s) + h*s;
    return S;

}

// -----------------------------------------------------------------//

 bool TestIfSE3(Matrix4d mat)
{
    if (abs(DistanceToSE3(mat)) < 1e-3)
        return true;
}

// -----------------------------------------------------------------//

 bool TestIfSO3(Matrix3d mat)
{
    if (abs(DistanceToSO3(mat)) < 1e-3)
        return true;
}

// -----------------------------------------------------------------//

 Matrix4d TransInv(Matrix4d T)
{
    auto [R,p] = TransToRp(T);
    Matrix4d invT;
    invT.block<3,3>(0,0) = R.transpose();
    invT.block<3,1>(0,3) = -R.transpose()*p;
    invT.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    return invT;
}

// -----------------------------------------------------------------//

 tuple<Matrix3d , Vector3d> TransToRp(Matrix4d T)
{
    return tuple<Matrix3d,Vector3d>(T.block<3,3>(0,0),T.block<3,1>(0,3));
}

// -----------------------------------------------------------------//

 Matrix4d VecTose3(VectorXd V)
{
    Matrix4d se3mat;
    se3mat.block<3,3>(0,0) = VecToso3(V.head(3));
    se3mat.block<3,1>(0,3) = V.tail(3);
    se3mat.block<1,4>(3,0) << 0 , 0 , 0 , 0;
    return se3mat;
}

// -----------------------------------------------------------------//

 Matrix3d VecToso3(Vector3d omg)
{
    Matrix3d so3mat;
    so3mat << 0            , -omg(2) , omg(1),
              omg(2) , 0             ,-omg(0),
              -omg(1), omg(0)  , 0           ;
    return so3mat;
}

// -----------------------------------------------------------------//

 MatrixXd adjoint(VectorXd V)
{
    MatrixXd adV(6,6);
    auto omgmat = VecToso3(V.head(3));
    adV.block<3,3>(0,0) = omgmat;
    adV.block<3,3>(0,3) = Matrix3d().setZero();
    adV.block<3,3>(3,0) = VecToso3(V.tail(3));
    adV.block<3,3>(3,3) = omgmat;

}

// -----------------------------------------------------------------//

 VectorXd se3ToVec(Matrix4d se3mat)
{
    VectorXd V(6);
    V(0) = se3mat(2,1); V(1) =  se3mat(0,2);
    V(2) = se3mat(1,0) ; V.tail(3) = se3mat.block<3,1>(0,3);
    return V;
}

// -----------------------------------------------------------------//

 Vector3d so3ToVec(Matrix3d so3mat)
{
    Vector3d omg;
    omg(0) = so3mat(2,1); omg(1) = so3mat(0,2);
    omg(2) = so3mat(1,0);
    return omg;
}