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

#include "stereorectify.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/optim.hpp>



/***
 * Computes the stereo rectification of two undistorted stereo cameras.
 * Output is in form of two homography matrices mapping points from image space
 * to rectified space of each camera.
 *
 * For the general algorithm, see:
 * E. Trucco, A. Verri, "Introductory Techniques for 3-D Computer Vision", page 160
 *
 * Additionally, the rectifying plane is rotated around the epipole so that the resulting
 * projective transformations as as close as possible (in algebraic sense) to an affine
 * transformation.
 *
 *  Parameters:
 *
 *             K0:  CAM0 intrinsic matrix
 *             K1:  CAM1 intrinsic matrix
 *            R,T:  Roto-translation transforming points from CAM1 to CAM0 reference frames
 * I0Size, I1Size:  CAM0 and CAM1 size respectively
 *        outSize:  Size of the rectified images
 *         H0, H1:  Output homographies to be applied to CAM0 and CAM1 respectively
 *            ROI:  Output common ROI between two cameras
 *
 *
 * Filippo Bergamasco 2018
 *
 */
void stereoRectifyUndistorted( const cv::Matx33d K0,
                               const cv::Matx33d K1,
                               const cv::Matx33d R,
                               const cv::Vec3d T,
                               const cv::Size I0Size,
                               const cv::Size I1Size,
                               const cv::Size outSize,
                               cv::Matx33d& H0, cv::Matx33d& H1,
                               cv::Rect& ROI )
{

    // Support class for rectification plane optimization
    class HFunctional : public cv::MinProblemSolver::Function
    {
    public:

        HFunctional( cv::Vec3d ep1,
                     cv::Matx33d _K0,
                     cv::Matx33d _K1,
                     cv::Matx33d _Ri ) : _H0( cv::Matx33d::eye() ), _H1( cv::Matx33d::eye() )
        {
            K0 = _K0;
            K0i = _K0.inv();
            K1i = _K1.inv();
            Ri = _Ri;

            Rv = ep1 / cv::norm( ep1 );
            N = Rv.cross( cv::Vec3d(0,1,0) ); N = N / cv::norm(N);
            Rk = Rv.cross(N);

            Rplane = cv::Matx33d( Rv(0), Rv(1), Rv(2), Rk(0), Rk(1), Rk(2), N(0), N(1), N(2) );
        }

        virtual double calc	(const double *x ) const
        {
            cv::Mat Radd = cv::Mat::zeros(3,1,CV_64FC1 );
            Radd.at<double>(0) = (*x)/180*3.14;
            cv::Rodrigues(Radd,Radd);

            _H0 =  cv::Matx33d(Radd) * Rplane * K0i;
            _H1 =  cv::Matx33d(Radd) * Rplane * Ri * K1i;

            _H0 *= 1.0/_H0(2,2);
            _H1 *= 1.0/_H1(2,2);

            double v1 =  _H0(2,0)*_H0(2,0) +  _H0(2,1)*_H0(2,1);
            double v2 =  _H1(2,0)*_H1(2,0) +  _H1(2,1)*_H1(2,1);

            // For numerical stability, H0 and H1 are normalized so that det(H)=1
            _H0 = _H0 * (1.0/std::cbrt(cv::determinant( _H0 )));
            _H1 = _H1 * (1.0/std::cbrt(cv::determinant( _H1 )));

            return std::max( v1 , v2 );
        }

        virtual int	getDims () const { return 2; }
        virtual void getGradient (const double *x, double *grad) {};
        virtual double getGradientEps () const { return 0; }

        cv::Matx33d H0() const { return _H0; }
        cv::Matx33d H1() const { return _H1; }

    private:
        cv::Vec3d Rv;
        cv::Vec3d N;
        cv::Vec3d Rk;

        cv::Matx33d K0;
        cv::Matx33d K0i;
        cv::Matx33d K1i;
        cv::Matx33d Ri;

        cv::Matx33d Rplane;
        mutable cv::Matx33d _H0;
        mutable cv::Matx33d _H1;
    };

    cv::Vec3d ep1 = T; // Epipole of camera 1

    //std::cout << ep1 << std::endl;

    cv::Ptr< HFunctional > hf( new HFunctional( ep1, K0, K1, R ) );
    cv::Ptr< cv::DownhillSolver > opt = cv::DownhillSolver::create( );
    opt->setFunction( hf );
    cv::Mat step=(cv::Mat_<double>(2,1)<<-0.5,-0.5);
    opt->setInitStep(step);
    opt->setTermCriteria( cv::TermCriteria( cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 500000, 1E-40 )  );

    //std::cout << "Optimizing best rectifying plane... ";
    cv::Mat x=(cv::Mat_<double>(2,1)<<0.0,0.0);
    opt->minimize( x );
    double best_angle = x.at<double>(0,0);
    //std::cout << "DONE" << std::endl << "Best angle: " << best_angle << " deg." << std::endl;

    hf->calc( &best_angle );
    H0 = hf->H0();
    H1 = hf->H1();
    //std::cout << "H0: " << std::endl << H0 << std::endl;
    //std::cout << "H1: " << std::endl << H1 << std::endl;

    const cv::Matx34d pts0( 0, I0Size.width, I0Size.width ,      0,
                            0,       0   ,   I0Size.height, I0Size.height,
                            1,       1   ,         1      ,      1 );
    const cv::Matx34d pts1( 0, I1Size.width, I1Size.width ,      0,
                            0,       0   ,   I1Size.height, I1Size.height,
                            1,       1   ,         1      ,      1 );

    cv::Matx34d ptsH0 = H0 * pts0;
    cv::Matx34d ptsH1 = H1 * pts1;
    for( size_t ii=0; ii<4; ++ii )
    {
        ptsH0(0,ii) /= ptsH0(2,ii);
        ptsH0(1,ii) /= ptsH0(2,ii);
        ptsH1(0,ii) /= ptsH1(2,ii);
        ptsH1(1,ii) /= ptsH1(2,ii);
    }
    cv::Rect_<double> rect0( cv::Point2d( cv::min( ptsH0(0,0), ptsH0(0,3) ), cv::min( ptsH0(1,0), ptsH0(1,1) ) ),
                    cv::Point2d( cv::max( ptsH0(0,1), ptsH0(0,2) ), cv::max( ptsH0(1,2), ptsH0(1,3) ) ) );
    cv::Rect_<double> rect1( cv::Point2d( cv::min( ptsH1(0,0), ptsH1(0,3) ), cv::min( ptsH1(1,0), ptsH1(1,1) ) ),
                    cv::Point2d( cv::max( ptsH1(0,1), ptsH1(0,2) ), cv::max( ptsH1(1,2), ptsH1(1,3) ) ) );

    //std::cout << "Rect0: " << rect0 << std::endl;
    //std::cout << "Rect1: " << rect1 << std::endl;

    const double top = cv::min( rect0.y, rect1.y );
    const double bottom = cv::max( rect0.y+rect0.height, rect1.y+rect1.height);
    //const double left = cv::min( rect0.x, rect1.x );
    //const double right = cv::max( rect0.x+rect0.width, rect1.x+rect1.width );

    //std::cout << top << " " << bottom << " " << left << " " << right << std::endl;

    cv::Matx33d Tr0( 1, 0, -rect0.x,
                     0, 1, -top,
                     0, 0,   1 );
    cv::Matx33d Sc0( outSize.width/(rect0.width), 0, 0,
                     0,outSize.height/(bottom-top), 0,
                     0, 0,   1 );
    cv::Matx33d Tr1( 1, 0, -rect1.x,
                     0, 1, -top,
                     0, 0,   1 );
    cv::Matx33d Sc1( outSize.width/(rect1.width), 0, 0,
                     0,outSize.height/(bottom-top), 0,
                     0, 0,   1 );

    H0 = Sc0*Tr0*H0;
    H1 = Sc1*Tr1*H1;
    H0 = H0 * (1.0/std::cbrt(cv::determinant( H0 )));
    H1 = H1 * (1.0/std::cbrt(cv::determinant( H1 )));

    // Compute ROI
    // NOTE: This is not the maximal common region
    //        but a close approximation
    //
    ptsH0 = H0 * pts0;
    ptsH1 = H1 * pts1;
    for( size_t ii=0; ii<4; ++ii )
    {
        ptsH0(0,ii) /= ptsH0(2,ii);
        ptsH0(1,ii) /= ptsH0(2,ii);
        ptsH1(0,ii) /= ptsH1(2,ii);
        ptsH1(1,ii) /= ptsH1(2,ii);
    }

    std::vector<double> xVEC(8);
    std::vector<double> yVEC(8);
    for( int ii=0; ii<8;  )
    {
        xVEC[ii] = ptsH0(0,ii/2);
        yVEC[ii] = ptsH0(1,ii/2);
        ii++;
        xVEC[ii] = ptsH1(0,ii/2);
        yVEC[ii] = ptsH1(1,ii/2);
        ii++;
    }
    std::sort( xVEC.begin(), xVEC.end() );
    std::sort( yVEC.begin(), yVEC.end() );

    ROI.x = xVEC[3];
    ROI.y = yVEC[3];
    ROI.width = xVEC[4]-ROI.x;
    ROI.height = yVEC[4]-ROI.y;
}


#ifdef STEREO_RECTIFY_TEST

double _randu(double a, double b)
{
    return cv::Matx12d::randu(a,b)(0,0);
}

cv::Mat create_image( cv::Size sz, std::vector< cv::Point2f > pts )
{
    cv::Mat img = cv::Mat::zeros( sz, CV_8UC3 );
    cv::rectangle( img, cv::Point(0,0), cv::Point( sz.width, sz.height ), cv::Scalar(50,50,50), -1 );

    for( size_t ii=0; ii<pts.size(); ++ii )
    {
        cv::drawMarker( img, pts[ii], cv::Scalar(0,0,250),cv::MARKER_TILTED_CROSS, 13, 2 );
    }

    return img;
}

void stereo_rectify_test()
{
    cv::theRNG().state = cv::getTickCount();
    cv::Size size0( _randu(1500,2500), _randu(1300,1700) );
    double f0 = _randu(1000,2000);
    cv::Matx33d K0( f0,  0, size0.width/2 + _randu(-100,100),
                     0, f0, size0.height/2 + _randu(-100,100),
                     0,  0, 1);

    std::cout << "K0: " << std::endl << K0 << std::endl;

    cv::Size size1( _randu(1500,2500), _randu(1300,1700) );
    double f1 = _randu(1000,2000);
    cv::Matx33d K1( f1,  0, size1.width/2 + _randu(-100,100),
                     0, f1, size1.height/2 + _randu(-100,100),
                     0,  0, 1);

    std::cout << "K1: " << std::endl << K1 << std::endl;

    // Generate random points
    cv::Mat pts(200,3, CV_32FC1 );
    cv::randu( pts, -50, 50 );
    pts.col(2) += 150;
    //std::cout << pts << std::endl;

    cv::Mat rvec0 = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat T0 = cv::Mat::zeros(3,1,CV_64FC1);

    cv::Mat rvec1 = cv::Mat::zeros(3,1,CV_64FC1);
    rvec1.at<double>(0) = _randu(0,0.5);
    rvec1.at<double>(1) = -1;
    rvec1.at<double>(2) = _randu(0,0.5);
    rvec1 = rvec1 / cv::norm(rvec1) * _randu(0,10) * 3.14 / 180;
    cv::Mat R;
    cv::Rodrigues( rvec1, R );

    cv::Mat T1 = cv::Mat::zeros(3,1,CV_64FC1);
    T1.at<double>(0) = 1;
    T1.at<double>(1) = _randu(-0.15, 0.15);
    T1.at<double>(2) = _randu(-0.15, 0.15);
    T1 = (T1 / cv::norm(T1));

    cv::Vec3d T = T1;

    cv::Mat pts0;
    cv::projectPoints( pts, rvec0, T0, K0, cv::Mat::zeros(5,1,CV_64FC1), pts0 );
    cv::Mat img0 = create_image( size0, pts0 );
    cv::imwrite("I0.png", img0);
    //std::cout << "Img0: " << std::endl << img0 << std::endl;

    std::cout << "T1: " << std::endl << T1 << std::endl;

    cv::Mat pts1;
    cv::projectPoints( pts, rvec1, T1, K1, cv::Mat::zeros(5,1,CV_64FC1), pts1 );
    cv::Mat img1 = create_image( size1, pts1 );
    cv::imwrite("I1.png", img1);
    //std::cout << "Img1: " << std::endl << img1 << std::endl;

    cv::Matx33d H0;
    cv::Matx33d H1;
    cv::Rect ROI;

    cv::Matx33d Rinv = R; Rinv = Rinv.t();
    cv::Vec3d Tinv = -Rinv * T;
    stereoRectifyUndistorted( K0, K1, Rinv, Tinv, size0, size1, size0, H0, H1, ROI );

    cv::Mat pts0r;
    cv::perspectiveTransform( pts0, pts0r, H0 );
    cv::Mat pts1r;
    cv::perspectiveTransform( pts1, pts1r, H1 );

    //std::cout << "Img0r: " << std::endl << img0r << std::endl;
    //std::cout << "Img1r: " << std::endl << img1r << std::endl;

    cv::Mat er =  (pts0r - pts1r);
    std::vector< cv::Mat > ch;
    cv::split(er, ch );
    std::cout << "Final RMS epipolar error: " << cv::norm(ch[1])/pts0r.rows << std::endl;

    cv::Mat I0r;
    cv::warpPerspective( img0, I0r, H0, size0 );
    cv::rectangle( I0r, ROI, cv::Scalar(255,255,255), 2 );
    cv::imwrite( "I0r.jpg", I0r );

    cv::Mat I1r;
    cv::warpPerspective( img1, I1r, H1, size0 );
    cv::rectangle( I1r, ROI, cv::Scalar(255,255,255), 2 );
    cv::imwrite( "I1r.jpg", I1r );


    cv::Mat stereo = cv::Mat::zeros( I0r.rows, I0r.cols + I1r.cols, CV_8UC3 );
    I0r.copyTo( stereo.colRange(0, I0r.cols) );
    I1r.copyTo( stereo.colRange(I0r.cols, I0r.cols + I1r.cols ) );
    cv::resize( stereo, stereo, cv::Size(), 0.5, 0.5 );
    cv::imwrite( "stereo.jpg", stereo );
}

#endif
