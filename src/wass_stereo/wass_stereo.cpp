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
#include <boost/shared_ptr.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WASS_ENABLE_OPTFLOW
#include <opencv2/optflow.hpp>
#endif

#include "incfg.hpp"
#include "wassglobal.hpp"
#include "log.hpp"
#include "utils.hpp"

#include "hires_timer.h"
#include "PovMesh.h"
#include "render.hpp"
#include "triangulate.hpp"
#include "nanoflann.hpp"

#include "PointPicker.hpp"

#include "stereorectify.h"


INCFG_REQUIRE( int, RANDOM_SEED, -1, "Random seed for ransac. -1 to use system timer" )
INCFG_REQUIRE( int, MIN_TRIANGULATED_POINTS, 100, "Minimum number of triangulated point to proceed with plane estimation" )
INCFG_REQUIRE( double, SAVE_INPUT_SCALE, 0.3, "Save a scaled version of input images (Set 1 to skip or a value <1 to specify scale ratio)" )
INCFG_REQUIRE( double, ZGAP_PERCENTILE, 99.0, "Z-gap percentile for outlier filtering" )
INCFG_REQUIRE( bool, DISABLE_AUTO_LEFT_RIGHT, false, "Disable automatic left-right detection" )
INCFG_REQUIRE( bool, SWAP_LEFT_RIGHT, false, "Swaps left-right images (only valid if DISABLE_AUTO_LEFT_RIGHT is set)" )
INCFG_REQUIRE( bool, SAVE_FULL_MESH, false, "Save 3D point cloud before plane outlier removal" )
INCFG_REQUIRE( int, PLANE_RANSAC_ROUNDS, 400, "number of RANSAC rounds for plane estimation" )
INCFG_REQUIRE( double, PLANE_RANSAC_THRESHOLD, 1.0, "RANSAC inlier threshold" )

INCFG_REQUIRE( double, PLANE_REFINE_XMIN, -9999, "Minimum point x-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_XMAX,  9999, "Maximum point x-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_YMIN, -9999, "Minimum point y-coordinate for plane refinement" )
INCFG_REQUIRE( double, PLANE_REFINE_YMAX,  9999, "Maximum point y-coordinate for plane refinement" )

INCFG_REQUIRE( double, PLANE_MAX_DISTANCE, 1.5, "Maximum point-plane distance allowed for the reconstructed point-cloud" )

INCFG_REQUIRE( bool, SAVE_AS_PLY, false, "Save final reconstructed point cloud also in PLY format" )
INCFG_REQUIRE( bool, SAVE_COMPRESSED, true, "Save in 16-bit compressed format" )

INCFG_REQUIRE( bool, USE_CUSTOM_STEREORECTIFY, false, "Use built-in stereorectify algorithm instead of the one provided by OpenCV" )
INCFG_REQUIRE( bool, DISABLE_RECTIFY_ROI, false, "Disable automatic ROI computation during stereo rectification (only enabled if USE_CUSTOM_STEREORECTIFY=true)" )

#ifdef WASS_ENABLE_OPTFLOW
INCFG_REQUIRE( bool, USE_OPTICAL_FLOW, false, "Use optical flow for 3D reconstruction (experimental)" )
INCFG_REQUIRE( int, FLOW_REFINEMENT_FULLRES_ITERATIONS, 200, "Number of iterations for flow refinement" )
INCFG_REQUIRE( double, FLOW_REFINEMENT_COLOR_CONSISTENCY_FACTOR, 100, "Color consistency factor for both the low-res and high-res flow refinement" )
INCFG_REQUIRE( double, FLOW_REFINEMENT_LOWRES_SMOOTHNESS_FACTOR, 90, "Smoothness factor for the low-res flow refinement" )
INCFG_REQUIRE( double, FLOW_REFINEMENT_FULLRES_SMOOTHNESS_FACTOR, 300, "Smoothness factor for the full-res flow refinement" )
INCFG_REQUIRE( int, FLOW_OPENING_DILATE, 1, "Dilate steps in flow mask" )
INCFG_REQUIRE( int, FLOW_OPENING_ERODE, 1, "Erode steps in flow mask" )
#endif

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



using namespace nanoflann;

template <typename T>
struct PointCloud
{
    struct Point
    {
        T  x,y;
        cv::Vec2f flow;
    };

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t size) const
    {
        const T d0=p1[0]-pts[idx_p2].x;
        const T d1=p1[1]-pts[idx_p2].y;

        if( size==1 )
            return d0*d0;

        return d0*d0+d1*d1;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x;
        else return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const { return false; }
};


typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor< float, PointCloud<float> > ,
                                  PointCloud< float >,
                                  2
                                  > my_kd_tree_t;


class KDTreeImpl
{
public:
    KDTreeImpl() : tree( 2, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) )
    {

    }
    PointCloud<float> cloud;
    my_kd_tree_t tree;
};




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

    int left_index;     // Index of the left camera
    int right_index;    // Index of the right camera

    double cam_distance;

    // Projection Matrix
    cv::Mat P0;
    cv::Mat P1;

    cv::Mat Rpose0;
    cv::Mat Tpose0;
    cv::Mat Rpose1;
    cv::Mat Tpose1;

    // Rectification data
    cv::Matx33d HL;
    cv::Matx33d HLi;
    cv::Matx33d HR;
    cv::Matx33d HRi;
    cv::Rect roi_comb_left;
    cv::Rect roi_comb_right;

    cv::Mat rec_R1;
    cv::Mat rec_R2;
    cv::Mat rec_P1;
    cv::Mat rec_P2;
    cv::Mat Q;
    cv::Mat imgleft_map1;
    cv::Mat imgleft_map2;
    cv::Mat imgright_map1;
    cv::Mat imgright_map2;

    // Stereo data
    cv::Mat disparity;
    class KDTreeImpl *pKDT_coarse_flow;
    cv::Mat coarse_flow_mask;
    double disparity_compensation;


    void swapLeftRight()
    {
        {
            int aux;
            aux = left_index;
            left_index = right_index;
            right_index = aux;
        }

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
        if( INCFG_GET( USE_CUSTOM_STEREORECTIFY ) )
        {
            cv::Vec3d uvr = (use_left ? HLi : HRi ) * cv::Vec3d( uv[0], uv[1], 1 );
            return cv::Vec2d( uvr[0] / uvr[2], uvr[1] / uvr[2] );
        }
        else
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
        env.left_index = 0;
        LOGI << "image 0 loaded, Size: " << env.left.cols << "x" << env.left.rows;
        env.right = cv::imread( (env.workdir / "undistorted/00000001.png").string(), cv::IMREAD_GRAYSCALE );
        env.right_index = 1;
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

    if( cv::abs( env.T.at<double>(1) ) >  cv::abs( env.T.at<double>(0) ) )
    {
        LOGE << "Vertical stereo not supported";
        return false;
    }

    LOGI << "Detected stereo setup:";
    if( env.T.at<double>(0) > 0 )
    {
        LOGI << "CAM1 (L) ---------  CAM0 (R)";
    }
    else
    {
        LOGI << "CAM0 (L) ---------  CAM1 (R)";
    }


    if( INCFG_GET(DISABLE_AUTO_LEFT_RIGHT) )
    {
        auto_swap=false;
        do_swap = INCFG_GET(SWAP_LEFT_RIGHT);
        LOGI << "auto left-right detection disabled. Swap left-right? " << (do_swap?"YES":"NO");

        if( do_swap )
        {
            LOGI << "swapping left-right images as requested";
            env.swapLeftRight();
        }
    }
    else
    {
        // Auto left-right swap
        if( env.T.at<double>(0) < 0 )
        {
            LOGI << "auto-swapping left-right images" << std::endl;
            env.swapLeftRight();
        }
    }

    cv::Size imgsize( env.left.cols, env.left.rows );
    cv::Rect ROI;

    if( INCFG_GET( USE_CUSTOM_STEREORECTIFY ) )
    {
        LOGI << "Using WASS custom stereorectify";

        stereoRectifyUndistorted( env.intrinsics_left, env.intrinsics_right, env.Rinv, env.Tinv, imgsize, imgsize, imgsize, env.HL, env.HR, ROI);

        env.HLi = env.HL.inv();
        env.HRi = env.HR.inv();

        if( env.left_index == 0 ) {
            WASS::save_matrix_txt<double>( (env.workdir / "H0_rect.txt").string(), cv::Mat(env.HL) );
            WASS::save_matrix_txt<double>( (env.workdir / "H1_rect.txt").string(), cv::Mat(env.HR) );
        } else {
            WASS::save_matrix_txt<double>( (env.workdir / "H1_rect.txt").string(), cv::Mat(env.HL) );
            WASS::save_matrix_txt<double>( (env.workdir / "H0_rect.txt").string(), cv::Mat(env.HR) );
        }

        cv::warpPerspective( env.left, env.left_rectified, env.HL, imgsize );
        cv::warpPerspective( env.right, env.right_rectified, env.HR, imgsize );

        env.roi_comb_left = ROI;//cv::Rect(0,0,env.left_rectified.cols, env.left_rectified.rows);//ROI;
        env.roi_comb_right = ROI;//cv::Rect(0,0,env.right_rectified.cols, env.right_rectified.rows);//ROI;

        if( INCFG_GET( DISABLE_RECTIFY_ROI ) )
        {
            env.roi_comb_left = cv::Rect(0,0,env.left_rectified.cols, env.left_rectified.rows);
            env.roi_comb_right =  cv::Rect(0,0,env.right_rectified.cols, env.right_rectified.rows);
        }

        env.left_crop = env.left_rectified(env.roi_comb_left).clone();
        env.right_crop = env.right_rectified(env.roi_comb_right).clone();
    }
    else
    {

        LOGI << "Rectifying via cv::stereoRectify";

        cv::Rect roi_left;
        cv::Rect roi_right;
        bool rectification_ok = false;

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

    }
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

INCFG_REQUIRE( int, DISPARITY_OFFSET, 0, "Offset in pixel to be applied. Positive: move right image to the right. Negative: move right image to the left")

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

INCFG_REQUIRE( int, DENSE_DISPARITY_BIGGEST_COMPONENT_THRESHOLD, 0, "Maximum squared gradient magnitude threshold for biggest connected component extraction (0 to disable)")


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

    const int disparity_offset = INCFG_GET( DISPARITY_OFFSET );
    LOGI << "Disparity offset: " << disparity_offset << " px";
    int disp_offset = 0;
    env.disparity_compensation = 0;
    if( disparity_offset > 0)
    {
        disp_offset = disparity_offset;
    }
    else
    {
        env.disparity_compensation = -disparity_offset;
    }

    //const int disp_offset = disparity_offset > 0 ? disparity_offset : 0;//INCFG_GET(DENSE_DISP_OFFSET);
    //LOGI << "disp offset: " << disp_offset << " (px)";
    //env.disparity_compensation = INCFG_GET(DIVERGENCE_COMPENSATION);
    //LOGI << "divergence compensation: " << env.disparity_compensation << " (px)";


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

#if 0
    /* Debug */
    left_image = left_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    right_image = right_image.colRange( numberOfDisparities, right_input.cols+numberOfDisparities );
    cv::imwrite((env.workdir/"stereo_L.png").string(),left_image);
    cv::imwrite((env.workdir/"stereo_R.png").string(),right_image);
    cv::imwrite((env.workdir/"stereo_D.png").string(),disparity);
    /**/
#endif

    //render_disparity16( env.workdir+"/disparity16.png", numberOfDisparities, minDisparity, disp_offset,disparity );

    cv::Mat disp_float = clean_and_convert_disparity( disparity, minDisparity, numberOfDisparities, disp_offset, 1.0/scale );
    WASS::Render::render_disparity_float( (env.workdir / "disparity_stereo_ouput.jpg").string(), disp_float);

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
    //
    //

    if( INCFG_GET(DISABLE_RECTIFY_ROI) && INCFG_GET(USE_CUSTOM_STEREORECTIFY ) )
    {
        cv::Mat mask = env.right_rectified==0;
        //disp_float_fullsize.setTo(0,mask);
    }

    aux = disp_float_fullsize.clone();
    if( INCFG_GET(MEDIAN_FILTER_WSIZE) >=3 )
    {
        LOGI << "applying median filter (window size " << INCFG_GET(MEDIAN_FILTER_WSIZE) << " px.)";
        cv::medianBlur( aux, disp_float_fullsize, INCFG_GET(MEDIAN_FILTER_WSIZE) );
    }

    if( INCFG_GET(DENSE_DISPARITY_BIGGEST_COMPONENT_THRESHOLD)>0 )
    {
        LOGI << "extracting the biggest connected component from the disparity map";
        LOGI << "assuming a sq gradient magnitude of " << INCFG_GET(DENSE_DISPARITY_BIGGEST_COMPONENT_THRESHOLD);
        cv::Mat gradX;
        cv::Mat gradY;
        cv::Sobel( disp_float_fullsize, gradX, CV_32FC1, 1, 0 );
        cv::Sobel( disp_float_fullsize, gradY, CV_32FC1, 0, 1 );
        cv::Mat gmag_sq = gradX.mul(gradX) + gradY.mul(gradY);

        cv::Mat large_grad = gmag_sq > INCFG_GET(DENSE_DISPARITY_BIGGEST_COMPONENT_THRESHOLD);
        cv::Mat dbg = cv::Mat::zeros( disp_float_fullsize.rows, disp_float_fullsize.cols, CV_8UC1 );
        dbg.setTo( 255, large_grad );
        cv::imwrite( ( env.workdir / "disparity_large_gradient.jpg").string(), dbg );

        disp_float_fullsize.setTo( 0.0f, large_grad );

        cv::Mat dispmask = disp_float_fullsize!=0;
        cv::Mat labels;
        cv::Mat biggestcomp_mask;
        cv::Mat stats;
        cv::Mat centroids;

        cv::connectedComponentsWithStats( dispmask, labels, stats, centroids );
        int max_area = 0;
        for( int i=1; i<stats.rows; ++i )
        {
            int area = stats.at< int >( i, cv::ConnectedComponentsTypes::CC_STAT_AREA );
            if( area>max_area )
            {
                max_area = area;
                biggestcomp_mask = (labels == i);
            }
        }
        dbg = cv::Mat::zeros( disp_float_fullsize.rows, disp_float_fullsize.cols, CV_8UC1 );
        dbg.setTo( 255, 1-biggestcomp_mask );
        cv::imwrite( ( env.workdir / "disparity_biggest_component.jpg").string(), dbg );

        disp_float_fullsize.setTo( 0.0f, 1-biggestcomp_mask );
    }

    aux = cv::Mat();

    env.disparity = cv::Mat::zeros( env.right_rectified.rows, env.right_rectified.cols, CV_32FC1 );
    disp_float_fullsize.copyTo( env.disparity(env.roi_comb_right) );


    //{
    //    save_matrix<float>("disp_raw.dat", env.disparity );
    //}

    // ------------------------------------------------
    // Debug rendering

    WASS::Render::render_disparity_float( (env.workdir / "disparity_final_scaled.jpg").string(), disp_float_fullsize );
    cv::Mat right_debug;
    cv::cvtColor( env.right_rectified.clone(),right_debug, cv::COLOR_GRAY2RGB);

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
INCFG_REQUIRE( std::string, LEFT_MASK_IMAGE, "none", "Filename of a (BW) left camera mask image. Note: File path is relative to current workdir. Use \"none\" for no mask" )
INCFG_REQUIRE( std::string, RIGHT_MASK_IMAGE, "none", "Filename of a (BW) right camera mask image. Note: File path is relative to current workdir. Use \"none\" for no mask" )
INCFG_REQUIRE( bool, DISCARD_BURNED_AREAS, true, "Discard white pixels (value>254)" )

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

    // Triangulation mask image
    cv::Mat left_mask = env.left.clone()*0 + 1;

    if( INCFG_GET( LEFT_MASK_IMAGE ) != std::string("none") )
    {
        const std::string filename = (env.workdir/INCFG_GET(LEFT_MASK_IMAGE)).string();
        LOGI << "Loading " << filename << " as left camera mask";
        cv::Mat aux = cv::imread( filename, cv::IMREAD_GRAYSCALE );
        if( aux.cols == left_mask.cols && aux.rows == left_mask.rows )
            cv::threshold(aux, left_mask, 0.5, 1, cv::THRESH_BINARY  );
        else
            LOGE << "not found or invalid image.";
    }
    if( INCFG_GET( DISCARD_BURNED_AREAS ) ) 
    {
        cv::Mat aux;
        cv::threshold(env.left, aux, 254.0, 1, cv::THRESH_BINARY  );
        left_mask = left_mask.mul(1-aux);
    }


    cv::Mat right_mask = env.right.clone()*0 + 1;
    if( INCFG_GET( RIGHT_MASK_IMAGE ) != std::string("none") )
    {
        const std::string filename = (env.workdir/INCFG_GET(RIGHT_MASK_IMAGE)).string();
        LOGI << "Loading " << filename << " as right camera mask";
        cv::Mat aux = cv::imread( filename, cv::IMREAD_GRAYSCALE );
        if( aux.cols == right_mask.cols && aux.rows == right_mask.rows )
            cv::threshold(aux, right_mask, 0.5, 1, cv::THRESH_BINARY  );
        else
            LOGE << "not found or invalid image.";
    }
    if( INCFG_GET( DISCARD_BURNED_AREAS ) ) 
    {
        cv::Mat aux;
        cv::threshold(env.right, aux, 254.0, 1, cv::THRESH_BINARY  );
        right_mask = right_mask.mul(1-aux);
    }

#ifdef WASS_ENABLE_OPTFLOW
    env.pKDT_coarse_flow = new KDTreeImpl();
    env.coarse_flow_mask = cv::Mat::zeros( env.right.rows, env.right.cols, CV_32FC1 );
#endif

    const int min_disp=1;
    size_t n_pts_triangulated = 0;
    env.mesh.reset( new PovMesh(env.roi_comb_right.width, env.roi_comb_right.height ) );


#if ENABLED(PLOT_3D_REPROJECTION)
    // Reproject debug
    cv::Matx34d Pc0 = env.P0;
    cv::Matx34d Pc1 = env.P1;
    cv::Mat dbg_P0 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC1 );
    cv::Mat dbg_P1 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC1 );
    cv::Mat dbg_R0 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC3 );
    cv::Mat dbg_R1 = cv::Mat::zeros( env.right.rows, env.right.cols, CV_8UC3 );

    const cv::Vec3b COLOR_CODE_POINT_OUTSIDE_IMAGE(255,255,0);  //Teal
    const cv::Vec3b COLOR_CODE_POINT_OUTSIDE_BBOX(0,255,255);   //Yellow
    const cv::Vec3b COLOR_CODE_POINT_TOO_CLOSE(255,0,0); //Blue
    const cv::Vec3b COLOR_CODE_POINT_TOO_DISTANT(0,0,255); //Red
    const cv::Vec3b COLOR_CODE_POINT_ANGLE_CHECK_FAIL(0,255,0); //Green
    const cv::Vec3b COLOR_CODE_POINT_HIGH_REPROJECTION_ERROR(255,255,255); //White

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

    int prog=0;
    int maxprog = env.roi_comb_right.height;
    int last_percent=0;
    const double stereo_scale = INCFG_GET(DENSE_SCALE);
    for( int yr_i=FIRSTROW; yr_i<env.roi_comb_right.y+env.roi_comb_right.height; yr_i++ )
    {
        for( int xr=env.roi_comb_right.x; xr<env.roi_comb_right.x+env.roi_comb_right.width; xr++ )
        {
            if( env.disparity.at<float>(yr_i,xr) > min_disp )
            {
                bool skip_triangulation = false;
                float xl = (float)( xr - env.roi_comb_right.x + env.roi_comb_left.x - env.disparity.at<float>(yr_i,xr) + env.disparity_compensation/stereo_scale );
                float yl = (float)yr_i;

                if( xl < 0 || xl >= env.left_rectified.cols )
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

                WASS::Render::show_image( WASS::Render::render_stereo(left_debug,right_debug), 0.3);
                //continue;
#endif
                const unsigned char val_r0 = env.right_rectified.at<unsigned char>(yr_i,xr);
                const unsigned char val_r1 = env.left_rectified.at<unsigned char>( std::floor(yl+0.5), std::floor(xl+0.5) );

                dbg_R0.at< cv::Vec3b >( yr_i,xr ) = cv::Vec3b(val_r0,val_r0,val_r0);
                dbg_R1.at< cv::Vec3b >( yr_i,xr ) = cv::Vec3b(val_r1,val_r1,val_r1);

                cv::Vec2d pi = env.unrectify( cv::Vec2d(xl,yl), true );
                cv::Vec2d qi = env.unrectify( cv::Vec2d(xr,yr_i), false );

                // Check if pi or qi falls outside the image (can happen when ROI is disabled)
                if( pi[0]<1 || pi[0]>=env.left.cols-1 || pi[1]<1 || pi[1]>=env.left.rows-1 ||  qi[0]<1 || qi[0]>=env.right.cols-1 || qi[1]<1 || qi[1]>=env.right.rows-1 )
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_IMAGE;
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_IMAGE;
                    skip_triangulation = true;
                }

                //std::cout << "pi: " << pi << std::endl << "qi: " << qi << std::endl;

                cv::Vec2d p( (pi[0]-env.intrinsics_left.at<double>(0,2)) / env.intrinsics_left.at<double>(0,0), (pi[1]-env.intrinsics_left.at<double>(1,2)) / env.intrinsics_left.at<double>(1,1) );
                cv::Vec2d q( (qi[0]-env.intrinsics_right.at<double>(0,2)) / env.intrinsics_right.at<double>(0,0),(qi[1]-env.intrinsics_right.at<double>(1,2)) / env.intrinsics_right.at<double>(1,1));


                if( pi[0] <= bbox_topleft[0]  || pi[1] <=bbox_topleft[1] ||
                    pi[0] >= bbox_botright[0] || pi[1] >=bbox_botright[1] )
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_BBOX;
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_BBOX;
                    skip_triangulation = true;
                }

                if( left_mask.at< unsigned char >( pi[1], pi[0] ) == 0 )
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_BBOX;
                    skip_triangulation = true;
                }

                if( right_mask.at< unsigned char >( qi[1], qi[0] ) == 0 )
                {
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_OUTSIDE_BBOX;
                    skip_triangulation = true;
                }


                // Angle check
                if( min_angle > 0 )
                {
                    cv::Vec3d d1 = cv::normalize( cv::Vec3d(p[0], p[1], 1.0) );
                    cv::Vec3d d2 = cv::normalize( (cv::Matx33d)(env.R)*cv::Vec3d(q[0], q[1], 1.0)+(cv::Vec3d)(env.T) );
                    double ang = fabs( acos( d1.ddot(d2) )*57.29577951 );
                    if( ang<min_angle )
                    {
                        dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_ANGLE_CHECK_FAIL;
                        dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_ANGLE_CHECK_FAIL;
                        skip_triangulation = true;
                    }
                }


                if( skip_triangulation )
                    continue;

                PointCloud<float>::Point newpoint;
                newpoint.x = std::floor( qi[0]+0.5f );
                newpoint.y = std::floor( qi[1]+0.5f );
                newpoint.flow = (pi-qi);

                //env.pKDT_coarse_flow->cloud.pts.push_back( newpoint );
                //env.coarse_flow_mask.at<float>( std::floor( qi[1] + 0.5 ), std::floor( qi[0]+0.5 ) ) = 1.0f;

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


#if 0
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
                if( reproj_error>1.0 ) // Maximum reprojection error allowed
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_HIGH_REPROJECTION_ERROR;
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_HIGH_REPROJECTION_ERROR;
                    continue;
                }
#endif


                // point distance check
                const double ptdistance = cv::norm( p3d );
                if( ptdistance < env.cam_distance/10.0 )
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_TOO_CLOSE;
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_TOO_CLOSE;
                    continue;
                }
                if( ptdistance > env.cam_distance*200.0)
                {
                    dbg_R0.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_TOO_DISTANT;
                    dbg_R1.at< cv::Vec3b >( yr_i,xr ) = COLOR_CODE_POINT_TOO_DISTANT;
                    continue;
                }

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
                    dbg_P0.at<unsigned char>( std::floor( p3d_reproj3(1)/p3d_reproj3(2) +0.5 ), std::floor(p3d_reproj3(0)/p3d_reproj3(2)+0.5) ) = (R+G+B)/3;
                }
                {
                    cv::Vec3d p3d_reproj3 = Pc1*cv::Vec4d( p3d(0),p3d(1),p3d(2),1 );
                    dbg_P1.at<unsigned char>( std::floor( p3d_reproj3(1)/p3d_reproj3(2) + 0.5), std::floor(p3d_reproj3(0)/p3d_reproj3(2)+0.5) ) = (R+G+B)/3;
                }
#endif

                n_pts_triangulated++;
            }
        }
        prog++;
        int percent = (int)((float)prog/(float)maxprog*100);
        if( percent-last_percent >= 15 )
        {
            last_percent = percent;
            LOGI << "... " << percent << "%";
        }
    }

    LOGI << "... 100%";
    LOGI << n_pts_triangulated << " valid points found";

#if ENABLED(PLOT_3D_REPROJECTION)
    //cv::imwrite( (env.workdir/"/undistorted/00000000_P0.jpg").string(), dbg_P0 );
    //cv::imwrite( (env.workdir/"/undistorted/00000001_P1.jpg").string(), dbg_P1 );
    cv::imwrite( (env.workdir/"/undistorted/R0.jpg").string(), dbg_R0 );
    cv::imwrite( (env.workdir/"/undistorted/R1.jpg").string(), dbg_R1 );
#endif

    return n_pts_triangulated;
}




#ifdef WASS_ENABLE_OPTFLOW

inline bool isFlowCorrect(cv::Point2f u)
{
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

static cv::Vec3b computeColor(float fx, float fy)
{
    static bool first = true;

    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static cv::Vec3i colorWheel[NCOLS];

    if (first)
    {
        int k = 0;

        for (int i = 0; i < RY; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255, 255 * i / RY, 0);

        for (int i = 0; i < YG; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255 - 255 * i / YG, 255, 0);

        for (int i = 0; i < GC; ++i, ++k)
            colorWheel[k] = cv::Vec3i(0, 255, 255 * i / GC);

        for (int i = 0; i < CB; ++i, ++k)
            colorWheel[k] = cv::Vec3i(0, 255 - 255 * i / CB, 255);

        for (int i = 0; i < BM; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255 * i / BM, 0, 255);

        for (int i = 0; i < MR; ++i, ++k)
            colorWheel[k] = cv::Vec3i(255, 0, 255 - 255 * i / MR);

        first = false;
    }

    const float rad = sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float)CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    cv::Vec3b pix;

    for (int b = 0; b < 3; b++)
    {
        const float col0 = colorWheel[k0][b] / 255.f;
        const float col1 = colorWheel[k1][b] / 255.f;

        float col = (1 - f) * col0 + f * col1;

        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= .75; // out of range

        pix[2 - b] = static_cast<uchar>(255.f * col);
    }

    return pix;
}


static void drawOpticalFlow(const cv::Mat_<cv::Point2f>& flow, cv::Mat& dst, float maxmotion = -1)
{
    dst.create(flow.size(), CV_8UC3);
    dst.setTo(cv::Scalar::all(0));

    // determine motion range:
    float maxrad = maxmotion;

    if (maxmotion <= 0)
    {
        maxrad = 1;
        for (int y = 0; y < flow.rows; ++y)
        {
            for (int x = 0; x < flow.cols; ++x)
            {
                cv::Point2f u = flow(y, x);

                if (!isFlowCorrect(u))
                    continue;

                maxrad = std::max<float>(maxrad, sqrt(u.x * u.x + u.y * u.y));
            }
        }
    }

    for (int y = 0; y < flow.rows; ++y)
    {
        for (int x = 0; x < flow.cols; ++x)
        {
            cv::Point2f u = flow(y, x);

            if (isFlowCorrect(u))
                dst.at<cv::Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
        }
    }
}


void flow_to_points( const cv::Mat& flow, const StereoMatchEnv& env, const cv::Mat& mask, cv::Mat& flow_absolute, cv::Mat& remap_img,  std::vector< cv::Point2d >& r_pts, std::vector< cv::Point2d >& l_pts )
{
    r_pts.clear();
    l_pts.clear();
    remap_img = 0;
    flow_absolute = 0;

    for( int i=0; i<flow.rows; ++i )
    {
        for( int j=0; j<flow.cols; ++j )
        {
            cv::Vec2f pxfl = flow.at< cv::Vec2f >(i,j);

            if( isFlowCorrect(pxfl) && cv::norm( pxfl ) < env.left.cols/2 && mask.at<float>(i,j)>0 )
            {
                cv::Vec2f fl_a = pxfl + cv::Vec2f(j,i);
                cv::Vec3f left_pt = cv::Vec3f( fl_a[0], fl_a[1], 1.0 );
                left_pt = left_pt / left_pt[2];

                if( left_pt[0]>0 && left_pt[0]<env.left.cols && left_pt[1]>0 && left_pt[1]<env.left.rows )
                {
                    unsigned char col_l = env.left.at<unsigned char>( std::floor(left_pt[1]+0.5), std::floor( left_pt[0]+0.5)  );
                    unsigned char col_r = env.right.at<unsigned char>(i,j);
                    if( col_l > 0 && col_r > 0 )
                    {
                        flow_absolute.at< cv::Vec2f >(i,j) = fl_a;
                        r_pts.push_back( cv::Point2d( j,i ) );
                        l_pts.push_back( cv::Point2d( left_pt[0], left_pt[1]) );

                        remap_img.at<unsigned char>(i,j)=col_l;
#if ENABLED(DEBUG_CORRESPONDENCES)
                        {
                            cv::Mat left_debug;
                            cv::Mat right_debug;
                            cv::cvtColor(env.left.clone(),left_debug, CV_GRAY2RGB);
                            cv::cvtColor(env.right.clone(),right_debug, CV_GRAY2RGB);

                            cv::circle( left_debug, l_pts.back() ,10,CV_RGB(255,0,0), 3 );
                            cv::circle( right_debug, r_pts.back() ,10,CV_RGB(255,0,0), 3 );

                            WASS::Render::show_image(  WASS::Render::render_stereo(left_debug,right_debug), 0.3);
                            //continue;
                        }
#endif
                    }
                }
            }
        }
    }
}


void vr_warpImage(cv::Mat &dst, cv::Mat &src, cv::Mat &flow_u, cv::Mat &flow_v)
{
    cv::Mat_<float> mapX = cv::Mat_<float>(flow_u.rows, flow_u.cols);
    cv::Mat_<float> mapY = cv::Mat_<float>(flow_v.rows, flow_v.cols);;

    for (int i = 0; i < flow_u.rows; i++)
    {
        float *pFlowU = flow_u.ptr<float>(i);
        float *pFlowV = flow_v.ptr<float>(i);
        float *pMapX = mapX.ptr<float>(i);
        float *pMapY = mapY.ptr<float>(i);
        for (int j = 0; j < flow_u.cols; j++)
        {
            pMapX[j] = j + pFlowU[j];
            pMapY[j] = i + pFlowV[j];
        }
    }
    remap(src, dst, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
}


bool refine_flow( StereoMatchEnv& env )
{
    cv::imwrite( (env.workdir/"flow_00_left.png").string(), env.left );
    cv::imwrite( (env.workdir/"flow_01_right.png").string(), env.right );

    cv::Mat flow = cv::Mat::zeros( env.right.rows, env.right.cols, CV_32FC2 );

    cv::Mat mask_opened = cv::Mat::ones( env.right.rows, env.right.cols, CV_32FC1);
    cv::dilate(env.coarse_flow_mask,mask_opened,cv::Mat(),cv::Point(-1,-1), INCFG_GET(FLOW_OPENING_DILATE) );
    cv::erode(mask_opened.clone(),mask_opened,cv::Mat(),cv::Point(-1,-1), INCFG_GET(FLOW_OPENING_ERODE) );

    {
        LOGI << "Building flow field index";
        env.pKDT_coarse_flow->tree.buildIndex();

        LOGI << "Interpolating flow field";

        for( int y=0; y<flow.rows; ++y )
        {
            for( int x=0; x<flow.cols; ++x )
            {
                float query_pt[2] = { (float)x, (float)y };
                const size_t num_results = 9;
                std::vector<size_t>   ret_index(num_results);
                std::vector<float> out_dist_sqr(num_results);
                env.pKDT_coarse_flow->tree.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

                cv::Vec2d fval=0;
                double wsum = 0;
                for( int IDX=0; IDX<num_results; ++IDX )
                {
                    const double dist = out_dist_sqr[IDX];
                    if( dist<1E-5 )
                    {
                        fval = env.pKDT_coarse_flow->cloud.pts[ ret_index[IDX] ].flow;
                        wsum = 1.0f;
                        break;
                    }

                    fval += env.pKDT_coarse_flow->cloud.pts[ ret_index[IDX] ].flow / dist;
                    wsum += (1.0f/dist);
                }

                flow.at< cv::Vec2f >( y,x ) = fval/wsum;
            }
        }

    }

    cv::Mat flow_render;
    drawOpticalFlow( flow, flow_render, 200 );
    cv::imwrite( (env.workdir/"flow_coarse.png").string(), flow_render );

#if 0
    {
        LOGI << "Interpolating missing flow data";

        std::vector< cv::Mat > flow_uv;
        cv::split( flow, flow_uv );

        cv::Mat original_flow_u = flow_uv[0].clone();
        cv::Mat original_flow_v = flow_uv[1].clone();

        cv::resize( original_flow_u, original_flow_u, cv::Size(), 0.25,0.25 );
        cv::resize( original_flow_v, original_flow_v, cv::Size(), 0.25,0.25 );
        cv::resize( flow_uv[0], flow_uv[0], cv::Size(), 0.25,0.25 );
        cv::resize( flow_uv[1], flow_uv[1], cv::Size(), 0.25,0.25 );

        cv::Mat coarse_mask_resized;
        cv::resize( env.coarse_flow_mask, coarse_mask_resized, cv::Size(), 0.25,0.25 );

        cv::Mat aux;
        for( int i=0; i<1000; ++i )
        {
            cv::blur(flow_uv[0],aux,cv::Size(3,3));
            flow_uv[0] = aux.mul(1-coarse_mask_resized) + original_flow_u.mul(coarse_mask_resized);

            cv::blur(flow_uv[1],aux,cv::Size(3,3));
            flow_uv[1] = aux.mul(1-coarse_mask_resized) + original_flow_v.mul(coarse_mask_resized);
        }

        cv::resize( flow_uv[0], flow_uv[0], cv::Size(), 4,4 );
        cv::resize( flow_uv[1], flow_uv[1], cv::Size(), 4,4 );
        /*
        cv::Mat mask_opened;
        cv::dilate(env.coarse_flow_mask,mask_opened,cv::Mat(),cv::Point(-1,-1),3);
        cv::erode(mask_opened.clone(),mask_opened,cv::Mat(),cv::Point(-1,-1),4);
        aux = aux+10000;
        flow_uv[0] = aux.mul(1-mask_opened) + flow_uv[0].mul(mask_opened);
        flow_uv[1] = aux.mul(1-mask_opened) + flow_uv[1].mul(mask_opened);
        */
        cv::merge( flow_uv,flow );
    }
#endif

    cv::Mat flow_absolute = cv::Mat::zeros( flow.rows,flow.cols, CV_32FC2);
    cv::Mat remap_img = cv::Mat::zeros( env.left.rows, env.left.cols, CV_8UC1 );
    std::vector< cv::Point2d > r_pts;
    std::vector< cv::Point2d > l_pts;
    flow_to_points( flow, env, mask_opened, flow_absolute, remap_img, r_pts, l_pts );
    cv::imwrite( (env.workdir/"flow_02_left_remapped_coarse.png").string(), remap_img );

#if 0
    {
        cv::Mat I0 = env.right.clone();
        cv::Mat I1 = env.left.clone();
        cv::Mat uv[2];
        cv::Mat flowMat = flow.clone();
        split(flowMat, uv);
        cv::Mat aaa;
        vr_warpImage( aaa, I1, uv[0], uv[1] );
        cv::imwrite( (env.workdir/"flow_02_left_remapped_coarse_w.png").string(), aaa );
        merge(uv, 2, flowMat);
    }
#endif

#if 1
    {
        LOGI << "Computing flow via Variational Refinement...";
        cv::Ptr< cv::optflow::VariationalRefinement > flow_ref = cv::optflow::createVariationalFlowRefinement();
        flow_ref->setDelta( INCFG_GET(FLOW_REFINEMENT_COLOR_CONSISTENCY_FACTOR) ); // Color consistency
        flow_ref->setGamma(0); // Gradient consistency
        flow_ref->setAlpha( INCFG_GET(FLOW_REFINEMENT_LOWRES_SMOOTHNESS_FACTOR) ); // Smoothness
        flow_ref->setFixedPointIterations(1500);

        cv::Mat I0;
        cv::Mat I1;
        cv::Mat flow_S[2];
        cv::split(flow, flow_S);

        {
            cv::resize( env.right, I0, cv::Size(), 0.25,0.25 );
            cv::resize( env.left, I1, cv::Size(), 0.25,0.25 );
            cv::resize( flow_S[0], flow_S[0], cv::Size(), 0.25,0.25 ); flow_S[0] *= 0.25f;
            cv::resize( flow_S[1], flow_S[1], cv::Size(), 0.25,0.25 ); flow_S[1] *= 0.25f;
            LOGI << "Low-res flow refinement";
            LOGI << "   Color consistency factor (delta):" << flow_ref->getDelta();
            LOGI << "Gradient consistency factor (gamma):" << flow_ref->getGamma();
            LOGI << "          Smoothness factor (alpha):" << flow_ref->getAlpha();
            LOGI << "                  Num of iterations:" << flow_ref->getFixedPointIterations();
            LOGI << "              Num of sor iterations:" << flow_ref->getSorIterations();
            flow_ref->calcUV( I0, I1, flow_S[0], flow_S[1] );
        }
        {
            cv::resize( flow_S[0], flow_S[0], cv::Size(flow.cols,flow.rows) ); flow_S[0] *= 4.0f;
            cv::resize( flow_S[1], flow_S[1], cv::Size(flow.cols,flow.rows) ); flow_S[1] *= 4.0f;
            flow_ref->setAlpha( INCFG_GET(FLOW_REFINEMENT_FULLRES_SMOOTHNESS_FACTOR) ); // Smoothness
            flow_ref->setFixedPointIterations( INCFG_GET(FLOW_REFINEMENT_FULLRES_ITERATIONS) );
            LOGI << "Full-res flow refinement";
            LOGI << "   Color consistency factor (delta):" << flow_ref->getDelta();
            LOGI << "Gradient consistency factor (gamma):" << flow_ref->getGamma();
            LOGI << "          Smoothness factor (alpha):" << flow_ref->getAlpha();
            LOGI << "                  Num of iterations:" << flow_ref->getFixedPointIterations();
            LOGI << "              Num of sor iterations:" << flow_ref->getSorIterations();
            flow_ref->calcUV( env.right, env.left, flow_S[0], flow_S[1] );
        }
        merge(flow_S, 2, flow);
        LOGI << "Done";
    }
#endif

    drawOpticalFlow( flow, flow_render, 200 );
    cv::imwrite( (env.workdir/"flow.png").string(), flow_render );

    flow_to_points( flow, env, mask_opened, flow_absolute, remap_img, r_pts, l_pts );
    cv::imwrite( (env.workdir/"flow_02_left_remapped_afine.png").string(), remap_img );


    LOGI << "Triangulating " << r_pts.size() << " points..";
    cv::Mat pt3D;
    cv::triangulatePoints( env.P0, env.P1, cv::Mat( l_pts ), cv::Mat( r_pts ), pt3D );


    env.mesh.reset( new PovMesh(env.right.cols, env.right.rows ) );
    for( size_t kk=0; kk<r_pts.size(); ++kk )
    {
        const cv::Point2d& rpt = r_pts[kk];
        const cv::Point2d& lpt = l_pts[kk];
        cv::Vec4d pt = pt3D.col( kk );

        pt = pt / pt[3];
        unsigned char R = env.right.at<unsigned char>( rpt.y, rpt.x );
        //unsigned char Rl = env.left.at<unsigned char>( std::floor(lpt.y+0.5), std::floor(rpt.x+0.5) );

        if( pt[2]<1.0 || pt[2]>100 )
            continue;

        env.mesh->set_point( rpt.x, rpt.y, cv::Vec3d( pt[0],pt[1],pt[2] ), R,R,R );
    }

    env.mesh->save_as_ply_points( (env.workdir/"/mesh_full_flow.ply").string() );

    return true;
}

#endif //Optical flow enabled


int save_configuration( std::string filename )
{
    WASS::setup_logger();
    LOG_SCOPE("wass_stereo");
    LOGI << "Writing " << filename;

    std::string cfg = incfg::ConfigOptions::instance().to_config_string();
    std::ofstream ofs( filename );
    if( !ofs.is_open() )
    {
        LOGE << "Unable to open " << filename << " for write";
        return -1;
    }
    ofs << cfg;
    ofs.close();

    LOGI << "Done!";
    return 0;
}


int main( int argc, char* argv[] )
{
    StereoMatchEnv env;
    WASS::exe_name_to_stdout( "wass_stereo" );

    if( argc == 1 )
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "wass_stereo [--genconfig] <config_file> <workdir> [--measure] [--rectify-only]" << std::endl << std::endl;
        std::cout << "Not enough arguments, aborting." << std::endl;
        return 0;
    }


    if( argc>1 && std::string("--genconfig").compare(  std::string(argv[1]) ) == 0 )
    {
        return save_configuration( "stereo_config.txt" );
    }


    if( argc != 3 && argc != 4 )
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
        LOGI << "Loading configuration file " << argv[1];

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

        if( save_configuration( (env.workdir/"stereo_config.txt").string() ) != 0 )
            LOGE << "Unable to save stereo configuration file";
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
        LOGI << "Reconstructing " << env.workdir;
        env.timer.start();
        env.cam_distance = 1.0;
        if( !load_data(env) )
        {
            return -1;
        }

        env.timer << "Data load";
        std::cout << "[P|10|100]" << std::endl;

        // Save projection matrices
        WASS::save_matrix_txt<double>( (env.workdir/"/P0cam.txt").string(), env.P0);
        WASS::save_matrix_txt<double>( (env.workdir/"/P1cam.txt").string(), env.P1);
        // Save poses
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam0_poseR.txt").string(), env.Rpose0);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam0_poseT.txt").string(), env.Tpose0);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam1_poseR.txt").string(), env.Rpose1);
        WASS::save_matrix_txt<double>( (env.workdir/"/Cam1_poseT.txt").string(), env.Tpose1);


        rectify(env);
        env.timer << "Rectification";
        std::cout << "[P|20|100]" << std::endl;

        // Save projection matrices again (may have been swapped by rectify)
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
            cv::cvtColor(env.left_rectified.clone(),l_temp, cv::COLOR_GRAY2RGB);
            cv::Mat r_temp;
            cv::cvtColor(env.right_rectified.clone(),r_temp, cv::COLOR_GRAY2RGB);
            cv::rectangle( l_temp, env.roi_comb_left, CV_RGB(255,0,0), 3 );
            cv::rectangle( r_temp, env.roi_comb_right, CV_RGB(255,0,0), 3 );

            cv::Mat Istereo = WASS::Render::render_stereo(l_temp,r_temp);
            for( size_t ii=0; ii<Istereo.rows; ii+=20 )
            {
                cv::line( Istereo, cv::Point(0,ii), cv::Point(Istereo.cols-1,ii), CV_RGB(255,0,0), 1 );
            }

            cv::imwrite( (env.workdir/"stereo.jpg").string(), Istereo );
        }

        if(  argc==4 && std::string("--rectify-only").compare(  std::string(argv[3]) ) == 0 )
        {
            LOGI << "All done.";
            return 0;
        }

        if(  argc==4 && std::string("--measure").compare(  std::string(argv[3]) ) == 0 )
        {

            auto select_and_triangulate = [ &env ] () {
                cv::Point2d pt_left, pt_right;

                {
                    PointPicker pp( "Left image", env.left );
                    pp.loop();
                    pt_left = pp.selected_point();
                    std::cout << "Left point: " << pt_left << std::endl;
                }

                {
                    PointPicker pp( "Right image", env.right );
                    pp.loop();
                    pt_right = pp.selected_point();
                    std::cout << "Right point: " << pt_right << std::endl;
                }

                cv::Vec2d p( (pt_left.x-env.intrinsics_left.at<double>(0,2)) / env.intrinsics_left.at<double>(0,0), (pt_left.y-env.intrinsics_left.at<double>(1,2)) / env.intrinsics_left.at<double>(1,1) );
                cv::Vec2d q( (pt_right.x-env.intrinsics_right.at<double>(0,2)) / env.intrinsics_right.at<double>(0,0),(pt_right.y-env.intrinsics_right.at<double>(1,2)) / env.intrinsics_right.at<double>(1,1));

                cv::Vec3d p1 = triangulate( p,q,env.R, env.T);
                return p1;
            };


            std::cout << "Pick first point" << std::endl;
            cv::Vec3d p1 = select_and_triangulate();
            std::cout << "P1: " << p1 << std::endl;
            std::cout << "Pick second point" << std::endl;
            cv::Vec3d p2 = select_and_triangulate();
            std::cout << "P2: " << p2 << std::endl;

            std::cout << "---------------------------------------------" << std::endl;
            std::cout << "Distance: " << cv::norm( p1-p2 ) << std::endl;

            return 0;
        }

        // Dense stereo
        sgbm_dense_stereo( env );
        env.timer << "Dense Stereo";
        std::cout << "[P|40|100]" << std::endl;

        // Triangulation
        size_t n_pts = triangulate(env);
        env.timer << "Triangulation";
        std::cout << "[P|60|100]" << std::endl;

#ifdef WASS_ENABLE_OPTFLOW
        if( INCFG_GET( USE_OPTICAL_FLOW) )
        {
            if( !refine_flow( env ) )
                return -1;
        }
#endif

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








