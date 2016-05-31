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


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "incfg.hpp"
#include "wassglobal.hpp"
#include "log.hpp"
#include "utils.hpp"

#include "hires_timer.h"
#include "PovMesh.h"
#include "render.hpp"
#include "triangulate.hpp"



INCFG_REQUIRE( int, RANDOM_SEED, -1, "Random seed for ransac. -1 to use system timer" )
INCFG_REQUIRE( int, MIN_TRIANGULATED_POINTS, 100, "Minimum number of triangulated point to proceed with plane estimation" )
INCFG_REQUIRE( double, SAVE_INPUT_SCALE, 0.3, "Save a scaled version of input images (Set 1 to skip or a value <1 to specify scale ratio)" )
INCFG_REQUIRE( double, ZGAP_PERCENTILE, 99.0, "Z-gap percentile for outlier filtering" )
INCFG_REQUIRE( bool, DISABLE_AUTO_LEFT_RIGHT, false, "Disable automatic left-right detection" )
INCFG_REQUIRE( bool, SWAP_LEFT_RIGHT, false, "Swaps left-right images (only valid if DISABLE_AUTO_LEFT_RIGHT is set)" )
INCFG_REQUIRE( bool, SAVE_FULL_MESH, false, "Save 3D point cloud before plane outlier removal" )
INCFG_REQUIRE( int, PLANE_RANSAC_ROUNDS, 200, "number of RANSAC rounds for plane estimation" )
INCFG_REQUIRE( int, PLANE_RANSAC_THRESHOLD, 1.0, "RANSAC inlier threshold" )

INCFG_REQUIRE( double, PLANE_REFINE_XMIN, -9999, "Minimum point x-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_XMAX,  9999, "Maximum point x-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_YMIN, -9999, "Minimum point y-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_YMAX,  9999, "Maximum point y-coordinate for plane refinement" )

INCFG_REQUIRE( int, PLANE_MAX_DISTANCE, 1.5, "Maximum point-plane distance allowed for the reconstructed point-cloud" )


INCFG_REQUIRE( bool, SAVE_AS_PLY, false, "Save final reconstructed point cloud also in PLY format" )
INCFG_REQUIRE( bool, SAVE_COMPRESSED, true, "Save in 16-bit compressed format" )

/*
 *  Compile-time feature set utilities
 */
#define ON  2-
#define OFF 1-
#define ENABLED(x) ( (x 0) == 2 )


/***********************************************************************/


#define DEBUG_CORRESPONDENCES       OFF
#define SKIP_TO_ROW                 OFF
#define SAVE_MESH_ALIGNED_AS_ASCII  OFF
#define ITERATIVE_TRIANGULATION     OFF
#define PLOT_3D_REPROJECTION        ON
#define GPU_BILATERAL               OFF


/***********************************************************************/


#if ENABLED(SKIP_TO_ROW)
#define FIRSTROW (env.roi_comb_right.y + 980)
#else
#define FIRSTROW (env.roi_comb_right.y)
#endif


cv::Mat stack_matrices( const cv::Mat& R, const cv::Mat& T )
{
    cv::Mat RT1 = cv::Mat::zeros(3,4,CV_64FC1);

    for(int i=0; i<3; ++i )
        for(int j=0; j<3; ++j )
            RT1.at<double>(i,j) = R.at<double>(i,j);

    for(int i=0; i<3; ++i )
        RT1.at<double>(i,3) = T.at<double>(i,0);

    return RT1;
}


void invert_RT( cv::Mat& R, cv::Mat& T )
{
    R = R.clone().t();
    T = -R*T;
}

struct StereoMatchEnv
{
    cvlab::HiresTimer timer;

    boost::filesystem::path workdir;
    cv::Mat intrinsics_left;
    cv::Mat intrinsics_right;
    cv::Mat undist;
    cv::Mat K0; // Intrinsics of Cam0
    cv::Mat K1; // Intrinsics of Cam1

    cv::Mat left;
    cv::Mat right;
    cv::Mat left_rectified;
    cv::Mat right_rectified;

    cv::Mat left_crop;
    cv::Mat right_crop;

    cv::Mat R;
    cv::Mat T;
    cv::Mat Rinv;
    cv::Mat Tinv;

    double cam_distance;

    // Projection Matrix
    cv::Mat P0;
    cv::Mat P1;

    cv::Mat Rpose0;
    cv::Mat Tpose0;
    cv::Mat Rpose1;
    cv::Mat Tpose1;

    // Rectification data
    cv::Mat rec_R1;
    cv::Mat rec_R2;
    cv::Mat rec_P1;
    cv::Mat rec_P2;
    cv::Mat Q;
    cv::Rect roi_comb_left;
    cv::Rect roi_comb_right;
    cv::Mat imgleft_map1;
    cv::Mat imgleft_map2;
    cv::Mat imgright_map1;
    cv::Mat imgright_map2;

    // Stereo data
    cv::Mat disparity;
    double disparity_compensation;


    void swapLeftRight()
    {
        cv::Mat aux;

        aux=left;
        left = right;
        right = aux;

        aux=intrinsics_left;
        intrinsics_left = intrinsics_right;
        intrinsics_right = aux;

        aux=R;
        R = Rinv;
        Rinv = aux;

        aux=T;
        T = Tinv;
        Tinv = aux;


        aux=Rpose0; Rpose0 = Rpose1; Rpose1=aux;
        aux=Tpose0; Tpose0 = Tpose1; Tpose1=aux;
        invert_RT( Rpose0, Tpose0 );
        invert_RT( Rpose1, Tpose1 );
        computeP();
    }

    inline cv::Vec2d unrectify( const cv::Vec2d& uv, const bool use_left ) const
    {
        const cv::Mat& intr = use_left? intrinsics_left : intrinsics_right;
        const cv::Matx33d R = use_left? (cv::Matx33d)rec_R1 : (cv::Matx33d)rec_R2;
        const cv::Mat& newintr = use_left ? rec_P1 : rec_P2;


        cv::Vec3d xyw = cv::Vec3d( (uv[0]-newintr.at<double>(0,2))/newintr.at<double>(0,0),
                                   (uv[1]-newintr.at<double>(1,2))/newintr.at<double>(1,1),
                                   1.0);

        xyw = R.t()*xyw;
        xyw[0]/=xyw[2];
        xyw[1]/=xyw[2];

        return cv::Vec2d( xyw[0]*intr.at<double>(0,0) + intr.at<double>(0,2),
                          xyw[1]*intr.at<double>(1,1) + intr.at<double>(1,2));
    }

    void computeP( )
    {
        P0=K0*stack_matrices(Rpose0,Tpose0);
        P1=K1*stack_matrices(Rpose1,Tpose1);
    }

    boost::shared_ptr< PovMesh > mesh;
};



bool load_data( StereoMatchEnv& env )
{
    LOG_SCOPE("load_data");
    env.R = WASS::load_matrix( env.workdir/"ext_R.xml" );

    if( env.R.rows != 3 || env.R.cols != 3 )
    {
        LOGE << "invalid extrinsic rotation matrix (ext_R.xml)";
        return false;
    }

    env.T = WASS::load_matrix( env.workdir/"ext_T.xml" );

    if( env.T.cols != 1 || env.T.rows != 3 )
    {
        LOGE << "invalid extrinsic translation vector (ext_T.xml)";
        return false;
    }

    env.Rinv = env.R.clone();
    env.Tinv = env.T.clone();
    invert_RT( env.Rinv, env.Tinv );

    const double required_cam_distance=env.cam_distance;
    double current_cam_distance = sqrt( env.T.at<double>(0,0)*env.T.at<double>(0,0) + env.T.at<double>(1,0)*env.T.at<double>(1,0) + env.T.at<double>(2,0)*env.T.at<double>(2,0) );

    // Scale T and Tinv
    env.T.at<double>(0,0) = env.T.at<double>(0,0)/current_cam_distance * required_cam_distance;
    env.T.at<double>(1,0) = env.T.at<double>(1,0)/current_cam_distance * required_cam_distance;
    env.T.at<double>(2,0) = env.T.at<double>(2,0)/current_cam_distance * required_cam_distance;

    env.Tinv.at<double>(0,0) = env.Tinv.at<double>(0,0)/current_cam_distance * required_cam_distance;
    env.Tinv.at<double>(1,0) = env.Tinv.at<double>(1,0)/current_cam_distance * required_cam_distance;
    env.Tinv.at<double>(2,0) = env.Tinv.at<double>(2,0)/current_cam_distance * required_cam_distance;

    env.undist = cv::Mat::zeros( 1,5,CV_32F);
    for( int i=0; i<5; i++ )
        env.undist.at<float>(0,i)=0.0;

    env.Rpose0 = cv::Mat::eye(3,3,CV_64FC1);
    env.Tpose0 = cv::Mat::zeros(3,1,CV_64FC1);
    env.Rpose1 = env.R.clone();
    env.Tpose1 = env.T.clone();

    // Load camera intrinsics parameters
    env.K0 = WASS::load_matrix( env.workdir / "intrinsics_00000000.xml" );
    env.intrinsics_left = env.K0.clone();

    env.K1 = WASS::load_matrix( env.workdir / "intrinsics_00000001.xml" );
    env.intrinsics_right = env.K1.clone();

    // Compute projection matrices
    env.computeP( );

    // Load images
    try {
        env.left = cv::imread( (env.workdir / "undistorted/00000000.png").string(), cv::IMREAD_GRAYSCALE );
        LOGI << "image 0 loaded, Size: " << env.left.cols << "x" << env.left.rows;
        env.right = cv::imread( (env.workdir / "undistorted/00000001.png").string(), cv::IMREAD_GRAYSCALE );
        LOGI << "image 1 loaded, Size: " << env.right.cols << "x" << env.right.rows;

        // Save a scaled version of the input images
        if( INCFG_GET(SAVE_INPUT_SCALE) < 1.0 )
        {
            size_t newwidth = static_cast<size_t>( env.left.cols * INCFG_GET(SAVE_INPUT_SCALE) );
            size_t newheight = static_cast<size_t>( env.left.rows * INCFG_GET(SAVE_INPUT_SCALE) );
            double scale = (double)newwidth / (double)env.left.cols;

            //Debug
            LOGI << "original size: " << env.left.cols << "x" << env.left.rows;
            LOGI << "  scaled size: " << newwidth << "x" << newheight;
            LOGI << "        scale: " << scale;

            cv::Mat left_small = env.left.clone();
            cv::resize( env.left, left_small, cv::Size(newwidth,newheight),0,0,cv::INTER_CUBIC);
            cv::imwrite( (env.workdir/"00000000_s.png").string(), left_small );
            cv::Mat right_small = env.right.clone();
            cv::resize( env.right, right_small, cv::Size(newwidth,newheight),0,0,cv::INTER_CUBIC);
            cv::imwrite( (env.workdir/"00000001_s.png").string(), right_small);

            cv::Mat intr_left_small = env.intrinsics_left.clone();
            intr_left_small *= scale;
            intr_left_small.at<double>(2,2) = 1;
            WASS::save_matrix_txt<double>( (env.workdir/"K0_small.txt").string(), intr_left_small );

            cv::Mat intr_right_small = env.intrinsics_right.clone();
            intr_right_small *= scale;
            intr_right_small.at<double>(2,2) = 1;
            WASS::save_matrix_txt<double>( (env.workdir/"K1_small.txt").string(), intr_right_small );

            std::ofstream ofs( (env.workdir/"scale.txt").c_str() );
            ofs.precision(16);
            ofs << std::scientific;
            ofs << scale;
            ofs.close();
        }

    } catch( cv::Exception e ) {
        LOGE << "unable to load input images";
        return false;
    }

    return true;
}




bool rectify( StereoMatchEnv& env )
{
    LOG_SCOPE("rectify");
    LOGI << "rectifying...";
    bool auto_swap = true;
    bool do_swap=false;

    if( INCFG_GET(DISABLE_AUTO_LEFT_RIGHT) )
    {
        auto_swap=false;
        do_swap = INCFG_GET(SWAP_LEFT_RIGHT);
        LOGI << "auto left-right detection disabled. Swap left-right? " << (do_swap?"YES":"NO");
    }

    bool rectification_ok = false;
    bool swapped = false;
    cv::Size imgsize( env.left.cols, env.left.rows );

    cv::Rect roi_left;
    cv::Rect roi_right;

    do
    {
        cv::stereoRectify( env.intrinsics_left, env.undist, env.intrinsics_right, env.undist, imgsize, env.R, env.T, env.rec_R1, env.rec_R2, env.rec_P1, env.rec_P2, env.Q, 0, 1.0, imgsize, &(roi_left), &(roi_right) );
        //std::cout << "P1: " << rec_P1 << std::endl;
        //std::cout << "P2: " << rec_P2 << std::endl;
        //std::cout << "Q: " << Q << std::endl;

        if( fabs(env.rec_P2.at<double>(0,3)) < fabs(env.rec_P2.at<double>(1,3)) ) {
            LOGE << "vertical stereo not supported";
            return false;
        }

        if( roi_left.width == 0 || roi_right.width==0 || roi_left.height==0 || roi_right.height == 0 ) {
            LOGE << "the epipole lies inside the image plane";
            return false;
        }

        if( auto_swap )
        {
            if( env.rec_P2.at<double>(0,3) < 0 )
            {
                //swap left/right
                LOGI << "auto-swapping left-right images" << std::endl;
                env.swapLeftRight();
            } else
            {
                rectification_ok = true;
            }
        }
        else
        {
            if( do_swap )
            {
                LOGI << "swapping left-right images as requested";
                env.swapLeftRight();
                do_swap=false;

            } else
            {
                rectification_ok=true;
            }

        }
    } while( !rectification_ok );


    int ymin = roi_left.y>roi_right.y?roi_left.y:roi_right.y;
    int ymax = (roi_left.y+roi_left.height)<(roi_right.y+roi_right.height)?(roi_left.y+roi_left.height):(roi_right.y+roi_right.height);

    env.roi_comb_left = cv::Rect( roi_left.x, ymin, roi_left.width, ymax-ymin);
    env.roi_comb_right = cv::Rect( roi_right.x, ymin, roi_right.width, ymax-ymin);

    if( env.roi_comb_left.width > env.roi_comb_right.width ) {
        // Crop left
        env.roi_comb_left.width = env.roi_comb_right.width;
    } else {
        // Crop right
        env.roi_comb_right.width = env.roi_comb_left.width;
    }


    cv::initUndistortRectifyMap( env.intrinsics_left, env.undist, env.rec_R1, env.rec_P1, imgsize, CV_32FC1, env.imgleft_map1, env.imgleft_map2 );
    cv::initUndistortRectifyMap( env.intrinsics_right, env.undist, env.rec_R2, env.rec_P2, imgsize, CV_32FC1, env.imgright_map1, env.imgright_map2 );

    cv::remap( env.left, env.left_rectified, env.imgleft_map1, env.imgleft_map2, cv::INTER_CUBIC );
    cv::remap( env.right, env.right_rectified, env.imgright_map1, env.imgright_map2, cv::INTER_CUBIC );


    env.left_crop = env.left_rectified(env.roi_comb_left).clone();
    env.right_crop = env.right_rectified(env.roi_comb_right).clone();

    LOGI << "rectification map generated";
    return true;
}



template <typename Mat_T>
void matrix_dilate_zero( const cv::Mat& src, cv::Mat& out )
{
    out = src.clone();
    const int nrows = src.rows;
    const int ncols = src.cols;

    for( int i=1; i<nrows-1; ++i  )
    {
        Mat_T* t = ((Mat_T*)src.ptr(i-1))+1;
        Mat_T* tm1 = t-1;
        Mat_T* tp1 = t+1;

        Mat_T* b = ((Mat_T*)src.ptr(i+1))+1;
        Mat_T* bm1 = b-1;
        Mat_T* bp1 = b+1;

        Mat_T* c = ((Mat_T*)src.ptr(i))+1;
        Mat_T* cm1 = c-1;
        Mat_T* cp1 = c+1;

        Mat_T* out_c = ((Mat_T*)out.ptr(i));
        for( int j=1; j<ncols-1; ++j )
        {

            if( *out_c==0 )
            {
               float avg=0; int avgnum=0;
               if( *tm1>0 ) { avg+=*tm1; ++avgnum; }
               if( *tp1>0 ) { avg+=*tp1; ++avgnum; }
               if( *t>0 ) { avg+=*t; ++avgnum; }
               if( *bm1>0 ) { avg+=*bm1; ++avgnum; }
               if( *bp1>0 ) { avg+=*bp1; ++avgnum; }
               if( *b>0 ) { avg+=*b; ++avgnum; }
               if( *cm1>0 ) { avg+=*cm1; ++avgnum; }
               if( *cp1>0 ) { avg+=*cp1; ++avgnum; }
               if( avgnum>1 )
                   *out_c = avg/(float)avgnum;
            }
            ++t; ++tm1; ++tp1;
            ++b; ++bm1; ++bp1;
            ++cm1; ++cp1;
            ++out_c;
        }
    }
}


template <typename Mat_T>
void matrix_erode_zero( const cv::Mat& src, cv::Mat& out )
{
    out = src.clone();
    const int nrows = src.rows;
    const int ncols = src.cols;

    for( int i=1; i<nrows-1; ++i  )
    {
        Mat_T* t = ((Mat_T*)src.ptr(i-1))+1;
        Mat_T* tm1 = t-1;
        Mat_T* tp1 = t+1;

        Mat_T* b = ((Mat_T*)src.ptr(i+1))+1;
        Mat_T* bm1 = b-1;
        Mat_T* bp1 = b+1;

        Mat_T* c = ((Mat_T*)src.ptr(i))+1;
        Mat_T* cm1 = c-1;
        Mat_T* cp1 = c+1;

        Mat_T* out_c = ((Mat_T*)out.ptr(i));
        *out_c = 0; ++out_c; // zeroes out the first column
        for( int j=1; j<ncols-1; ++j )
        {
            *out_c = (*t==0) || (*tm1==0) || (*tp1==0) || (*b==0) || (*bm1==0) || (*bp1==0) || (*cm1==0) || (*cp1==0) ? 0 : *out_c;
            ++t; ++tm1; ++tp1;
            ++b; ++bm1; ++bp1;
            ++cm1; ++cp1;
            ++out_c;
        }
        *out_c = 0; ++out_c; // zeroes out the last column
    }


    // zeroes out the first and the last row
    Mat_T* out_c_first = ((Mat_T*)out.ptr(0));
    Mat_T* out_c_last = ((Mat_T*)out.ptr(nrows-1));
    for( int j=0; j<ncols; ++j )
    {
        *out_c_first = 0;
        *out_c_last = 0;
        ++out_c_first;
        ++out_c_last;
    }

}


inline cv::Mat clean_and_convert_disparity( const cv::Mat disp, const int mindisp, const int numberOfDisparities, const int disp_offset, const double scale )
{
    cv::Mat dispout = cv::Mat::zeros( disp.rows, disp.cols, CV_32FC1 );

    for( int i=0; i<dispout.rows; ++i )
    {
        for( int j=0; j<dispout.cols; ++j )
        {

            float dval = ((float)disp.at< signed short >(i,j))/16.0f;
            if( dval <= mindisp || dval > numberOfDisparities )
                continue;

            dval += disp_offset;
            dispout.at<float>(i,j) = (float)( dval*scale );
        }
    }

    return dispout;
}



/*******************************************************************************
 *    DENSE STEREO
 *
 *******************************************************************************/

INCFG_REQUIRE( int, MIN_DISPARITY, 1, "Minimum disparity allowed (in px)")
INCFG_REQUIRE( int, MAX_DISPARITY, 640, "Maximum disparity allowed")
INCFG_REQUIRE( int, WINSIZE, 13, "Stereo match window size")
INCFG_REQUIRE( double, DENSE_SCALE, 1.0, "Image resize before dense stereo")

INCFG_REQUIRE( int, DENSE_DISP_OFFSET, 0, "Offset in pixel to be applied")
INCFG_REQUIRE( int, DIVERGENCE_COMPENSATION, 0, "Offset in pixel to be applied")

INCFG_REQUIRE( int, DISP_DILATE_STEPS, 1, "Number of dilate steps to be applied to the disparity map")
INCFG_REQUIRE( int, DISP_EROSION_STEPS, 2, "Number of erosion steps to be applied to the disparity map")
INCFG_REQUIRE( int, MEDIAN_FILTER_WSIZE, 0, "Disparity median filter window size (0 to disable)")

INCFG_REQUIRE( int, DENSE_P1_MULT, 2, "SGBM P1 parameter")
INCFG_REQUIRE( int, DENSE_P2_MULT, 64, "SGBM P2 parameter")
INCFG_REQUIRE( int, DENSE_UNIQUENESS_RATIO, 1, "SGBM Uniqueness ratio")
INCFG_REQUIRE( int, DENSE_DISP12MAXDIFF, -1, "SGBM Disp12MaxDiff")
INCFG_REQUIRE( int, DENSE_PREFILTER_CAP, 60, "SGBM PreFilterCap")
INCFG_REQUIRE( int, DENSE_SPECKLE_RANGE, 16, "SGBM SpeckleRange")
INCFG_REQUIRE( int, DENSE_SPECKLE_WINDOW_SIZE, -70, "SGBM SpeckleWindowSize")

void sgbm_dense_stereo( StereoMatchEnv& env )
{
    LOG_SCOPE("sgbm_dense_stereo");
    const int minDisparity = INCFG_GET(MIN_DISPARITY);
    const int numberOfDisparities = INCFG_GET(MAX_DISPARITY);
    const int winsize = INCFG_GET(WINSIZE);

    cv::Mat aux;
    const int P1 = INCFG_GET(DENSE_P1_MULT)*winsize*winsize;
    const int P2 = INCFG_GET(DENSE_P2_MULT)*winsize*winsize;

    cv::Ptr<cv::StereoSGBM> dense_stereo =  cv::StereoSGBM::create( minDisparity, numberOfDisparities, winsize, P1, P2 );

    //dense_stereo.fullDP = true;
    dense_stereo->setUniquenessRatio( INCFG_GET(DENSE_UNIQUENESS_RATIO) );
    dense_stereo->setDisp12MaxDiff( INCFG_GET(DENSE_DISP12MAXDIFF) );
    dense_stereo->setPreFilterCap( INCFG_GET(DENSE_PREFILTER_CAP) );
    dense_stereo->setSpeckleRange( INCFG_GET(DENSE_SPECKLE_RANGE) );
    dense_stereo->setSpeckleWindowSize( INCFG_GET(DENSE_SPECKLE_WINDOW_SIZE) ) ;
    const double scale = INCFG_GET(DENSE_SCALE);

    cv::Mat left_input;
    cv::Mat right_input;
    cv::resize( env.right_crop, right_input, cv::Size(),scale,scale,cv::INTER_CUBIC);
    cv::resize( env.left_crop, left_input, cv::Size(),scale,scale,cv::INTER_CUBIC);

    // we have to pad left and right images to numberOfDisparities rows to avoid opencv dense stereo bug

    const int disp_offset = INCFG_GET(DENSE_DISP_OFFSET);
    LOGI << "disp offset: " << disp_offset << " (px)";

    env.disparity_compensation = INCFG_GET(DIVERGENCE_COMPENSATION);
    LOGI << "divergence compensation: " << env.disparity_compensation << " (px)";


    int stereo_img_width = left_input.cols+numberOfDisparities+disp_offset;
    cv::Mat left_image = cv::Mat::zeros( left_input.rows, stereo_img_width,  CV_8UC1 );
    cv::Mat target_left = left_image.colRange(numberOfDisparities+disp_offset-env.disparity_compensation, stereo_img_width-env.disparity_compensation );
    left_input.copyTo( target_left );

    cv::Mat right_image = cv::Mat::zeros( right_input.rows, stereo_img_width, CV_8UC1 );
    //cv::Mat target_right = right_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    //right_input.copyTo( target_right );

    cv::Mat target_right = right_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    cv::Mat source_right = right_input.colRange(0,right_input.cols);
    source_right.copyTo( target_right );

    cv::imwrite( (env.workdir/"/stereo_input.jpg").string(), WASS::Render::render_stereo_vertical(left_image,right_image) );

    cv::Mat disparity;
    LOGI << "computing dense disparity map... (may take a while)";
    dense_stereo->compute( right_image, left_image, disparity );

    disparity = disparity.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );

    /* Debug
    left_image = left_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    right_image = right_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    cv::imwrite(env.workdir+"/stereo_L.png",left_image);
    cv::imwrite(env.workdir+"/stereo_R.png",right_image);
    cv::imwrite(env.workdir+"/stereo_D.png",disparity);
    */

    //render_disparity16( env.workdir+"/disparity16.png", numberOfDisparities, minDisparity, disp_offset,disparity );

    cv::Mat disp_float = clean_and_convert_disparity( disparity, minDisparity, numberOfDisparities, disp_offset, 1.0/scale );
    WASS::Render::render_disparity_float( (env.workdir / "disparity_stereo_ouput.png").string(), disp_float);

    // DILATE
    const int disp_dilate_steps = INCFG_GET(DISP_DILATE_STEPS);

    if( disp_dilate_steps > 0 )
    {
        LOGI << "applying dilate filter (" << disp_dilate_steps << " steps)";
        for( int step=1; step<=disp_dilate_steps; ++step )
        {
            aux = disp_float.clone();
            //cv::dilate( aux, disp_float, cv::Mat(3,3,CV_8U),cv::Point(-1,-1), disp_dilate_steps );
            matrix_dilate_zero<float>(aux, disp_float );
        }
        //render_disparity_float(env.workdir+"/disparity_stereo_dilate.png",disp_float);
    }
    else
    {
        LOGI << "dilate filter skipped.";
    }

    // ERODE
    const int disp_erosion_steps = INCFG_GET( DISP_EROSION_STEPS );

    if( disp_erosion_steps > 0 )
    {
        LOGI << "applying erode filter (" << disp_erosion_steps << " steps)";
        for( int step=1; step<=disp_erosion_steps; ++step )
        {
            aux = disp_float.clone();
            //cv::erode( aux, disp_float, cv::Mat(3,3,CV_8U),cv::Point(-1,-1), disp_erode_steps );
            matrix_erode_zero<float>( aux, disp_float );
        }
        //render_disparity_float(env.workdir+"/disparity_stereo_erode.png",disp_float);
    }
    else
    {
        LOGI << "erode filter skipped.";
    }


    cv::Mat disp_float_fullsize;
    cv::Mat disp_float_fullsize_nn;

    // disparity map is resized using both nearest neighbour and bicubic filter.
    // All pixels in which the values in the resized nearest neighbour map are not
    // valid are also set to invalid in the bicubic map.
    // this partially ensures that good values are not interpolated with bad ones.

    cv::resize( disp_float, disp_float_fullsize_nn, env.roi_comb_right.size(), 0, 0, cv::INTER_NEAREST );
    cv::resize( disp_float, disp_float_fullsize, env.roi_comb_right.size(), 0, 0, cv::INTER_CUBIC );


    // erode disparity map, just to be sure to retain only good values from cubic map
    aux = disp_float_fullsize_nn.clone();
    //cv::erode( aux, disp_float_fullsize_nn, cv::Mat(3,3,CV_8U),cv::Point(-1,-1), 1 );
    matrix_erode_zero<float>( aux, disp_float_fullsize_nn );
    aux = cv::Mat();

    // remove bad disparities after resize
    for( size_t i=0; i<disp_float_fullsize.rows; ++i )
    {
        float* pSrcNN = (float*)disp_float_fullsize_nn.ptr((int)i);
        float* pDest = (float*)disp_float_fullsize.ptr((int)i);

        for( size_t j=0; j<disp_float_fullsize_nn.cols; ++j )
        {
            if( *pSrcNN == 0 )
            {
                *pDest=0.0f;
            }
            pSrcNN++;
            pDest++;
        }
    }

    // Now in disp_float_fullsize we have a good resized disparity map.
    aux = disp_float_fullsize.clone();
    if( INCFG_GET(MEDIAN_FILTER_WSIZE) >=3 )
    {
        LOGI << "applying median filter (window size " << INCFG_GET(MEDIAN_FILTER_WSIZE) << " px.)";
        cv::medianBlur( aux, disp_float_fullsize, INCFG_GET(MEDIAN_FILTER_WSIZE) );
    }

    aux = cv::Mat();

    env.disparity = cv::Mat::zeros( env.right_rectified.rows, env.right_rectified.cols, CV_32FC1 );
    disp_float_fullsize.copyTo( env.disparity(env.roi_comb_right) );

    //{
    //    save_matrix<float>("disp_raw.dat", env.disparity );
    //}

    // ------------------------------------------------
    // Debug rendering

    WASS::Render::render_disparity_float( (env.workdir / "disparity_final_scaled.png").string(), disp_float_fullsize );
    cv::Mat right_debug;
    cv::cvtColor( env.right_rectified.clone(),right_debug, CV_GRAY2RGB);

    for( int i=0; i<right_debug.rows; ++i )
    {
        for( int j=0; j<right_debug.cols; ++j )
        {
            unsigned char v = right_debug.at<cv::Vec3b>(i,j)[1];
            v = std::min<unsigned char>(255,env.disparity.at<float>(i,j)>1?100:v);
            right_debug.at<cv::Vec3b>(i,j)[1]=v;
        }
    }

    cv::rectangle( right_debug, env.roi_comb_right, CV_RGB(255,0,0), 3 );
    cv::resize( right_debug, right_debug, cv::Size(), 0.5,0.5,cv::INTER_LINEAR );
    cv::imwrite( ( env.workdir / "disparity_coverage.jpg").string(), right_debug );

    LOGI << "dense stereo completed successfully";
}




/*******************************************************************************
 *    3D TRIANGULATION
 *
 *******************************************************************************/

INCFG_REQUIRE( double, TRIANG_MIN_ANGLE, 20.0, "Minimum ray angle for triangulation (in degrees)")
INCFG_REQUIRE( double, TRIANG_BBOX_TOP, -1.0, "Triangulation bounding box top coordinate in px wrt. the left image (-1 to disable)")
INCFG_REQUIRE( double, TRIANG_BBOX_LEFT, -1.0, "Triangulation bounding box left coordinate in px wrt. the left image (-1 to disable)")
INCFG_REQUIRE( double, TRIANG_BBOX_RIGHT, -1.0, "Triangulation bounding box right coordinate in px wrt. the left image (-1 to disable)")
INCFG_REQUIRE( double, TRIANG_BBOX_BOTTOM, -1.0, "Triangulation bounding box bottom coordinate in px wrt. the left image (-1 to disable)")

size_t triangulate( StereoMatchEnv& env )
{
    LOG_SCOPE("triangulate");
    const double min_angle = INCFG_GET(TRIANG_MIN_ANGLE);

    // Triangulation bounding box (in pixels) wrt. left image

    cv::Vec2d bbox_topleft(0,0);
    cv::Vec2d bbox_botright( env.left.cols, env.left.rows );

    if( INCFG_GET(TRIANG_BBOX_TOP) >= 0 && INCFG_GET(TRIANG_BBOX_LEFT) >= 0 &&
        INCFG_GET(TRIANG_BBOX_BOTTOM) >= 0 && INCFG_GET(TRIANG_BBOX_RIGHT) >=0 )
    {
        bbox_topleft = cv::Vec2d(INCFG_GET(TRIANG_BBOX_LEFT), INCFG_GET(TRIANG_BBOX_TOP));
        bbox_botright = cv::Vec2d(INCFG_GET(TRIANG_BBOX_RIGHT), INCFG_GET(TRIANG_BBOX_BOTTOM) );
    }

    const int min_disp=1;
    size_t n_pts_triangulated = 0;
    env.mesh.reset( new PovMesh(env.roi_comb_right.width, env.roi_comb_right.height ) );


#if ENABLED(PLOT_3D_REPROJECTION)
    // Reproject debug
    cv::Matx34d Pc0 = env.P0;
    cv::Matx34d Pc1 = env.P1;
    cv::Mat dbg_P0 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC1 );
    cv::Mat dbg_P1 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC1 );

#endif

#if ENABLED(ITERATIVE_TRIANGULATION)
    cv::Matx34d PP0 = cv::Matx34d::eye();
    cv::Matx34d PP1;
    for( int i=0; i<3; ++i )
        for( int j=0; j<3; ++j)
             PP1(i,j) = env.R.at<double>(i,j);
    PP1(0,3)=env.T.at<double>(0,0);
    PP1(1,3)=env.T.at<double>(1,0);
    PP1(2,3)=env.T.at<double>(2,0);
#endif

    /*
    cv::Matx33d left_Kinv = cv::Matx33d::eye();
    left_Kinv(0,0) = 1.0 / env.intrinsics_left.at<double>(0,0);
    left_Kinv(1,1) = 1.0 / env.intrinsics_left.at<double>(1,1);
    left_Kinv(0,2) = -env.intrinsics_left.at<double>(0,2) / env.intrinsics_left.at<double>(0,0);
    left_Kinv(1,2) = -env.intrinsics_left.at<double>(1,2) / env.intrinsics_left.at<double>(1,1);
    */

#ifdef EXPERIMENTAL_PLANE_TRIANGULATION_CONSTRAINT
    cv::Matx41d plane;
    cv::Vec3d plane_n;
    cv::Vec3d plane_p;
    {
        std::ifstream ifs( env.workdir+"/plane.txt" );
        if( !ifs.fail() )
        {
            std::cout << "Loading plane" << std::endl;
            ifs >> plane(0);
            ifs >> plane(1);
            ifs >> plane(2);
            ifs >> plane(3);
            std::cout << plane << std::endl;

            plane_n = cv::Vec3d(plane(0), plane(1), plane(2) );
            plane_p = cv::Vec3d(0,0,-plane(3)/plane(2));

            std::cout << "Plane normal:" << plane_n << std::endl;
            std::cout << "Plane point:" << plane_p << std::endl;

        }
    }
#endif

    LOGI << "triangulating disparity map";

    double mean_reproj_error = 0.0;

    int prog=0;
    int maxprog = env.roi_comb_right.height;
    int last_percent=0;
    for( int yr_i=FIRSTROW; yr_i<env.roi_comb_right.y+env.roi_comb_right.height; yr_i++ )
    {
        for( int xr=env.roi_comb_right.x; xr<env.roi_comb_right.x+env.roi_comb_right.width; xr++ )
        {
            if( env.disparity.at<float>(yr_i,xr) > min_disp )
            {
                float xl = (float)( xr - env.roi_comb_right.x + env.roi_comb_left.x - env.disparity.at<float>(yr_i,xr) + env.disparity_compensation );
                float yl = (float)yr_i;

                if( xl < 0 || xl >= env.imgleft_map1.cols )
                    continue;

#if ENABLED(DEBUG_CORRESPONDENCES)
                cv::Mat left_debug;
                cv::Mat right_debug;
                cv::cvtColor(env.left_rectified.clone(),left_debug, CV_GRAY2RGB);
                cv::cvtColor(env.right_rectified.clone(),right_debug, CV_GRAY2RGB);

                {
                    for( int i=0; i<right_debug.rows; ++i )
                    {
                        for( int j=0; j<right_debug.cols; ++j )
                        {
                            unsigned char v = right_debug.at<cv::Vec3b>(i,j)[1];
                            v = std::min<unsigned char>(255,env.disparity.at<float>(i,j)>min_disp?100:v);
                            right_debug.at<cv::Vec3b>(i,j)[1]=v;
                        }
                    }
                }

                cv::rectangle( left_debug, env.roi_comb_left, CV_RGB(255,0,0), 3 );
                cv::rectangle( right_debug, env.roi_comb_right, CV_RGB(255,0,0), 3 );

                cv::circle( left_debug, cv::Point(xl,yl),10,CV_RGB(255,0,0), 3 );
                cv::circle( right_debug, cv::Point(xr,yr_i),10,CV_RGB(255,0,0), 3 );

                StereoMatch::Render::show_image( render_stereo(left_debug,right_debug), 0.3);
                //continue;
#endif

                cv::Vec2d pi = env.unrectify( cv::Vec2d(xl,yl), true );
                cv::Vec2d qi = env.unrectify( cv::Vec2d(xr,yr_i), false );

                cv::Vec2d p( (pi[0]-env.intrinsics_left.at<double>(0,2)) / env.intrinsics_left.at<double>(0,0), (pi[1]-env.intrinsics_left.at<double>(1,2)) / env.intrinsics_left.at<double>(1,1) );
                cv::Vec2d q( (qi[0]-env.intrinsics_right.at<double>(0,2)) / env.intrinsics_right.at<double>(0,0),(qi[1]-env.intrinsics_right.at<double>(1,2)) / env.intrinsics_right.at<double>(1,1));

                if( pi[0] <= bbox_topleft[0]  || pi[1] <=bbox_topleft[1] ||
                    pi[0] >= bbox_botright[0] || pi[1] >=bbox_botright[1] )
                    continue;

                // Angle check
                if( min_angle > 0 )
                {
                    cv::Vec3d d1 = cv::normalize( cv::Vec3d(p[0], p[1], 1.0) );
                    cv::Vec3d d2 = cv::normalize( (cv::Matx33d)(env.R)*cv::Vec3d(q[0], q[1], 1.0)+(cv::Vec3d)(env.T) );
                    double ang = fabs( acos( d1.ddot(d2) )*57.29577951 );
                    if( ang<min_angle )
                    {
                        continue;
                    }
                }

#if ENABLED(ITERATIVE_TRIANGULATION)
                cv::Vec3d p3d = IterativeLinearLSTriangulation(cv::Point3d(p[0],p[1],p[2]),PP0,cv::Point3d(q[0],q[1],q[2]),PP1);
#else
                cv::Vec3d p3d = triangulate(p,q,env.R,env.T);
#endif


#ifdef EXPERIMENTAL_PLANE_TRIANGULATION_CONSTRAINT
                // Compute plane intersection
                cv::Vec3d l0(0,0,0);
                cv::Vec3d l = cv::Vec3d(p(0),p(1),1); l=l/cv::norm(l);
                double dd = (plane_p - l0).ddot( plane_n ) / l.ddot(plane_n);
                cv::Vec3d inters = l0+dd*l;


                // Project to the intersection normal
                p3d = 0.0*p3d + 1.0*( inters + plane_n * plane_n.ddot(p3d-inters) );
#endif


                /* Reprojection error */
                cv::Vec2d p3d_reproj0( p3d[0]/p3d[2] * env.intrinsics_left.at<double>(0,0) + env.intrinsics_left.at<double>(0,2) ,
                                      p3d[1]/p3d[2] * env.intrinsics_left.at<double>(1,1) + env.intrinsics_left.at<double>(1,2) );
                //cv::Vec3d p3d_reproj3 = Pc0*cv::Vec4d( p3d(0),p3d(1),p3d(2),1 );
                //cv::Vec2d p3d_reproj( p3d_reproj3(0)/p3d_reproj3(2), p3d_reproj3(1)/p3d_reproj3(2) );
                /*
                std::cout << p3d_reproj << std::endl;
                std::cout << pi << std::endl;
                std::cout << cv::norm(pi-p3d_reproj) << std::endl;
                */
                double reproj_error = cv::norm(pi-p3d_reproj0);
                if( reproj_error>0.5 ) // Maximum reprojection error allowed
                    continue;
                /**/

                // New iterative method (not working right now)
                //cv::Mat_<double> Xpt = IterativeLinearLSTriangulation(cv::Point3d(p[0],p[1],1.0), Pc0, cv::Point3d(q[0],q[1],1.0), Pc1 );
                //cv::Vec3d p3d(Xpt(0),Xpt(1),Xpt(2));
                //cv::Mat_<double> Xpt = IterativeLinearLSTriangulation(cv::Point3d(pi[0],pi[1],1.0), Pc0, cv::Point3d(qi[0],qi[1],1.0), Pc1 );
                //cv::Vec3d p3d(Xpt(0),Xpt(1),Xpt(2));

                // point distance check
                const double ptdistance = cv::norm( p3d );
                if( ptdistance < env.cam_distance/10.0 ||  ptdistance > env.cam_distance*200.0)
                    continue;


                mean_reproj_error += reproj_error;
                const unsigned char R = env.right.at<unsigned char>( (int)qi[1], (int)qi[0] );
                const unsigned char G = R;
                const unsigned char B = R;
                env.mesh->set_point( xr-env.roi_comb_right.x,
                                     yr_i-env.roi_comb_right.y,
                                     p3d,
                                     R,
                                     G,
                                     B);

#if ENABLED(PLOT_3D_REPROJECTION)
                {
                    cv::Vec3d p3d_reproj3 = Pc0*cv::Vec4d( p3d(0),p3d(1),p3d(2),1 );
                    dbg_P0.at<unsigned char>( p3d_reproj3(1)/p3d_reproj3(2), p3d_reproj3(0)/p3d_reproj3(2) ) = (R+G+B)/3;
                }
                {
                    cv::Vec3d p3d_reproj3 = Pc1*cv::Vec4d( p3d(0),p3d(1),p3d(2),1 );
                    dbg_P1.at<unsigned char>( p3d_reproj3(1)/p3d_reproj3(2), p3d_reproj3(0)/p3d_reproj3(2) ) = (R+G+B)/3;
                }
#endif

                n_pts_triangulated++;
            }
        }
        prog++;
        int percent = (int)((float)prog/(float)maxprog*100);
        if( percent-last_percent>=15 )
        {
            last_percent = percent;
            LOGI << "... " << percent << "%";
        }
    }
    mean_reproj_error /= (double)n_pts_triangulated;

    LOGI << "... 100%";
    LOGI << n_pts_triangulated << " valid points found";
    LOGI << "average reprojection error: " << mean_reproj_error << " (px)";

#if ENABLED(PLOT_3D_REPROJECTION)
    cv::imwrite( (env.workdir/"/undistorted/00000000_P0.png").string(), dbg_P0 );
    cv::imwrite( (env.workdir/"/undistorted/00000001_P1.png").string(), dbg_P1 );
#endif

    return n_pts_triangulated;
}




int main( int argc, char* argv[] )
{
    StereoMatchEnv env;
    WASS::exe_name_to_stdout( "wass_stereo" );

    if( argc == 1 )
	{
        std::cout << "Usage:" << std::endl;
        std::cout << "wass_stereo [--genconfig] <config_file> <workdir>" << std::endl << std::endl;
		std::cout << "Not enough arguments, aborting." << std::endl;
		return -1;
	}


    if( argc>1 && std::string("--genconfig").compare(  std::string(argv[1]) ) == 0 )
    {
        WASS::setup_logger();
        LOG_SCOPE("wass_stereo");
        LOGI << "Generating stereo_config.txt ...";

        std::string cfg = incfg::ConfigOptions::instance().to_config_string();

        std::ofstream ofs( "stereo_config.txt" );
        if( !ofs.is_open() )
        {
            LOGE << "Unable to open stereo_config.txt for write";
            return -1;
        }
        ofs << cfg;
        ofs.close();

        LOGI << "Done!";
        return 0;
    }

    if( argc != 3 )
    {
        std::cerr << "Invalid arguments" << std::endl;
        return -1;
    }

    // Workdir check
    env.workdir = boost::filesystem::path( argv[2] );
    if( !boost::filesystem::exists( env.workdir) )
    {
        std::cerr << env.workdir << " does not exists, aborting." << std::endl;
        return -1;
    }

    WASS::setup_logger( (env.workdir/"wass_stereo_log.txt").string() );
    LOG_SCOPE("wass_stereo");

    {
        // Config file load
        LOGI << "Loding configuration file " << argv[1];

        std::ifstream ifs( argv[1] );
        if( !ifs.is_open() )
        {
            LOGE << "Unable to load " << argv[1];
            return -1;
        }


        try
        {
            incfg::ConfigOptions::instance().load( ifs );

        } catch( std::runtime_error& er )
        {
            LOGE << er.what();
            return -1;
        }
    }


    // Initialize random seed
    if( INCFG_GET(RANDOM_SEED) == -1 )
    {
        srand( (unsigned int)time(0) );
    }
    else
    {
        srand( INCFG_GET(RANDOM_SEED) );
        LOGI << "random seed set to: " << INCFG_GET(RANDOM_SEED);
    }

    try
    {
        env.timer.start();
        env.cam_distance = 1.0;
        if( !load_data(env) )
        {
            return -1;
        }

        env.timer << "Data load";
        std::cout << "[P|10|100]" << std::endl;

        rectify(env);
        env.timer << "Rectification";
        std::cout << "[P|20|100]" << std::endl;


        // Save projection matrices
        WASS::save_matrix_txt<double>( (env.workdir/"/P0cam.txt").string(), env.P0);
        WASS::save_matrix_txt<double>( (env.workdir/"/P1cam.txt").string(), env.P1);

        // Save poses
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam0_poseR.txt").string(), env.Rpose0);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam0_poseT.txt").string(), env.Tpose0);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam1_poseR.txt").string(), env.Rpose1);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam1_poseT.txt").string(), env.Tpose1);


        {
            // Output debug
            cv::Mat l_temp;
            cv::cvtColor(env.left_rectified.clone(),l_temp, CV_GRAY2RGB);
            cv::Mat r_temp;
            cv::cvtColor(env.right_rectified.clone(),r_temp, CV_GRAY2RGB);
            cv::rectangle( l_temp, env.roi_comb_left, CV_RGB(255,0,0), 3 );
            cv::rectangle( r_temp, env.roi_comb_right, CV_RGB(255,0,0), 3 );
            cv::imwrite( (env.workdir/"stereo.jpg").string(), WASS::Render::render_stereo(l_temp,r_temp) );
        }

        // Dense stereo
        sgbm_dense_stereo( env );
        env.timer << "Dense Stereo";
        std::cout << "[P|40|100]" << std::endl;

        // Triangulation
        size_t n_pts = triangulate(env);
        env.timer << "Triangulation";
        std::cout << "[P|60|100]" << std::endl;

        if( n_pts < INCFG_GET(MIN_TRIANGULATED_POINTS) )
        {
            LOGE << "Too few points triangulated, aborting";
            return -1;
        }

        /* debug
        if( !env.mesh->save_as_ply_points( (env.workdir+"/mesh_post_stereo.ply") ) )
        {
            std::cout << "Unable to save mesh data." << std::endl;
            return -1;
        }
        */

#if 0
        //cv::imwrite(env.workdir+"/LEFT.jpg", env.left_crop);
        //cv::imwrite(env.workdir+"/RIGHT.jpg", env.right_crop);

        // CROP mesh, if needed
        int crop_top=0;
        int crop_left=0;
        int crop_bottom=env.mesh->height();
        int crop_right=env.mesh->width();
        bool docrop=false;
        if( opts->optionExist("CROP_TOP") )
        {
            crop_top = opts->get("CROP_TOP").as_int(); docrop=true;
        }
        if( opts->optionExist("CROP_LEFT") )
        {
            crop_left = opts->get("CROP_LEFT").as_int(); docrop=true;
        }
        if( opts->optionExist("CROP_BOTTOM") )
        {
            crop_bottom = opts->get("CROP_BOTTOM").as_int(); docrop=true;
        }
        if( opts->optionExist("CROP_RIGHT") )
        {
            crop_right = opts->get("CROP_RIGHT").as_int(); docrop=true;
        }

        if( docrop )
        {
            std::cout << "Cropping mesh to [" << crop_left << "," << crop_top << "] - [" << crop_right << "," << crop_bottom << "]" << std::endl;
            env.mesh->crop(crop_top,crop_left,crop_bottom,crop_right);
        } else
        {
            std::cout << "No mesh cropping specified, use CROP_TOP,CROP_LEFT,CROP_RIGHT,CROP_BOTTOM to override" << std::endl;
        }
#endif

        //env.mesh->save_as_triangulated_ply( (env.workdir/"/mesh_triang_before.ply").string(), 10000.1  );

        double percentile = env.mesh->compute_zgap_percentile( INCFG_GET(ZGAP_PERCENTILE) );
        env.timer << "Z-gap stats";
        env.mesh->cluster_biggest_connected_component( env.workdir, percentile );
        env.timer << "Outlier removal";
        std::cout << "[P|80|100]" << std::endl;
        //env.mesh->save_as_triangulated_ply( (env.workdir/"/mesh_triang_after.ply").string(), 10000.1  );

        if( INCFG_GET(SAVE_FULL_MESH) )
        {
            if( !env.mesh->save_as_ply_points( (env.workdir/"/mesh_full.ply").string() ) )
            {
                LOGE << "unable to save mesh data.";
                return -1;
            }
        }

        LOGI << "estimating best fitting plane...";
        if( !env.mesh->ransac_find_plane( INCFG_GET(PLANE_RANSAC_ROUNDS), INCFG_GET(PLANE_RANSAC_THRESHOLD) ) )
        {
            LOGE << "ransac failed.";
            return -1;
        }
        env.timer << "Plane fitting";
        std::cout << "[P|90|100]" << std::endl;


        LOGI << "refining plane";

        env.mesh->crop_plane( INCFG_GET(PLANE_RANSAC_THRESHOLD) );

        std::vector< cv::Vec3d > dbg_inliers;
        env.mesh->refine_plane( INCFG_GET(PLANE_REFINE_XMIN), INCFG_GET(PLANE_REFINE_XMAX),
                                INCFG_GET(PLANE_REFINE_YMIN), INCFG_GET(PLANE_REFINE_YMAX),
                                &dbg_inliers );

        {
            std::ofstream ofs( (env.workdir/"/plane_refinement_inliers.xyz").c_str() );
            for( size_t i=0; i<dbg_inliers.size(); i+=10 )
            {
                ofs << dbg_inliers[i][0] << " " << dbg_inliers[i][1] << " " << dbg_inliers[i][2] << std::endl;
            }
            ofs.flush();
            ofs.close();
        }


        env.mesh->crop_plane( INCFG_GET(PLANE_MAX_DISTANCE) ); // Crop the mesh again since the parameters are changed
        env.timer << "Plane refinement";

        LOGI << "exporting point cloud data";

        if( INCFG_GET(SAVE_AS_PLY) )
        {
            if( !env.mesh->save_as_ply_points( (env.workdir/"/mesh.ply").string() ) )
            {
                LOGE << "unable to save mesh data.";
                return -1;
            }
        }

        if( INCFG_GET(SAVE_COMPRESSED) )
        {
            if( !env.mesh->save_as_xyz_compressed( (env.workdir/"/mesh_cam.xyzC").string() )  )
            {
                LOGE << "unable to save mesh data";
                return -1;
            }
        } else
        {
            if( !env.mesh->save_as_xyz_binary( (env.workdir/"/mesh_cam.xyzbin").string() ) )
            {
                LOGE << "unable to save mesh data";
                return -1;
            }
        }


        // Save plane parameters
        std::vector<double> plane_coeffs = env.mesh->get_plane_params();
        {
            std::ofstream ofs( (env.workdir/"/plane.txt").c_str() );
            ofs << std::setprecision(20);
            for( int i=0; i<4; ++i)
                ofs << plane_coeffs[i] << std::endl;
            ofs.flush();
            ofs.close();
        }


        env.timer.stop();
        std::cout << "[P|100|100]" << std::endl;
        WASS::Render::show_time_stats( env.timer );
        LOGI << "All done.";
    }
    catch( std::runtime_error& e)
    {
        LOGE << e.what();
        return -1;
    }

}








