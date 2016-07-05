/*************************************************************************

WASS - Wave Acquisition Stereo System
Copyright (C) 2016  Filippo Bergamasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*************************************************************************/

#ifndef _TRIANGULATE_H_
#define _TRIANGULATE_H_
#include <opencv2/opencv.hpp>


inline cv::Vec3d triangulate( cv::Vec2d p, cv::Vec2d q, const cv::Mat R, const cv::Mat T)
{
    double Af[4*3];
    double Bf[4*1];

    double A[9];
    double b[3];
    double x[3];

    // prepare Af
    Af[0] = -1.0;   Af[1] = 0.0;    Af[2] = p[0];
    Af[3] = 0.0;    Af[4] = -1.0;   Af[5] = p[1];
    Af[6] = q[0]*R.at<double>(2,0)-R.at<double>(0,0);    Af[7] = q[0]*R.at<double>(2,1)-R.at<double>(0,1);   Af[8] = q[0]*R.at<double>(2,2)-R.at<double>(0,2);
    Af[9] = q[1]*R.at<double>(2,0)-R.at<double>(1,0);    Af[10] = q[1]*R.at<double>(2,1)-R.at<double>(1,1);   Af[11] = q[1]*R.at<double>(2,2)-R.at<double>(1,2);

    // prepare Bf
    Bf[0]=0.0;
    Bf[1]=0.0;
    Bf[2]=T.at<double>(0,0)-T.at<double>(2,0)*q[0];
    Bf[3]=T.at<double>(1,0)-T.at<double>(2,0)*q[1];


    // Compute A= Af'Af
    A[0] = Af[0]*Af[0] + Af[3]*Af[3] + Af[6]*Af[6] + Af[9]*Af[9];
    A[1] = Af[0]*Af[1] + Af[3]*Af[4] + Af[10]*Af[9] + Af[6]*Af[7];
    A[2] = Af[0]*Af[2] + Af[3]*Af[5] + Af[11]*Af[9] + Af[6]*Af[8];

    A[3] = A[1];
    A[4] = Af[1]*Af[1] + Af[10]*Af[10] + Af[4]*Af[4] + Af[7]*Af[7];
    A[5] = Af[10]*Af[11] + Af[1]*Af[2] + Af[4]*Af[5] + Af[7]*Af[8];

    A[6] = A[2];
    A[7] = A[5];
    A[8] = Af[11]*Af[11] + Af[2]*Af[2] + Af[5]*Af[5] + Af[8]*Af[8];

    // Compute b = Af'*Bf
    b[0] = Af[0]*Bf[0] + Af[3]*Bf[1] + Af[6]*Bf[2] + Af[9]*Bf[3];
    b[1] = Af[1]*Bf[0] + Af[10]*Bf[3] + Af[4]*Bf[1] + Af[7]*Bf[2];
    b[2] = Af[2]*Bf[0] + Af[11]*Bf[3] + Af[5]*Bf[1] + Af[8]*Bf[2];

    cv::Mat Amat(3,3,CV_64FC1,A);
    cv::Mat Bmat(3,1,CV_64FC1,b);
    cv::Mat Xmat(3,1,CV_64FC1,x);
    cv::solve( Amat, Bmat, Xmat, cv::DECOMP_LU );
    return cv::Vec3d(x[0], x[1], x[2]);

}


/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
inline cv::Vec3d LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
                   cv::Matx34d P,						  //camera 1 matrix
                   cv::Point3d u1,						  //homogenous image point in 2nd camera
                   cv::Matx34d P1						  //camera 2 matrix
                                   )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
	cv::Mat B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
                      -(u.y*P(2,3)  -P(1,3)),
                      -(u1.x*P1(2,3)    -P1(0,3)),
                      -(u1.y*P1(2,3)    -P1(1,3)));
 
    cv::Mat X;
    cv::solve(A,B,X,cv::DECOMP_SVD);
 
    return cv::Vec3d(X.at<double>(0,0), X.at<double>(1,0), X.at<double>(2,0));
}


/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
inline cv::Vec3d IterativeLinearLSTriangulation( cv::Point3d u,    //homogenous image point (u,v,1)
												 cv::Matx34d P,          //camera 1 matrix
												 cv::Point3d u1,         //homogenous image point in 2nd camera
												 cv::Matx34d P1          //camera 2 matrix
                                            ) {
    double wi = 1, wi1 = 1;

    cv::Mat_<double> X(4,1);
    cv::Vec3d X_ = LinearLSTriangulation(u,P,u1,P1);
    X(0) = X_[0]; X(1) = X_[1]; X(2) = X_[2]; X(3) = 1.0;

    for (int i=0; i<5; i++) { //Hartley suggests 10 iterations at most
         
        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
         
        //breaking point
        //if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }

    return cv::Vec3d(X.at<double>(0,0), X.at<double>(1,0), X.at<double>(2,0));
}

inline cv::Vec3d triangulate( cv::Vec2d pi, cv::Vec2d qi, cv::Mat K1, cv::Mat K2, cv::Mat R, cv::Mat T)
{
    cv::Vec2d p( (pi[0]-K1.at<double>(0,2)) / K1.at<double>(0,0), (pi[1]-K1.at<double>(1,2)) / K1.at<double>(1,1) );
    cv::Vec2d q( (qi[0]-K2.at<double>(0,2)) / K2.at<double>(0,0),(qi[1]-K2.at<double>(1,2)) / K2.at<double>(1,1));
    return triangulate( p,q,R,T);
}


#endif
