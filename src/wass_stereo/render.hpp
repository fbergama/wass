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


#ifndef _RENDER_H_
#define _RENDER_H_

#include "hires_timer.h"
#include <opencv2/core.hpp>

namespace WASS
{
namespace Render
{

inline void render_disparity( const std::string filename, const int numberOfDisparities, const cv::Mat& disp)
{
    cv::Mat disprender = cv::Mat::zeros( disp.rows, disp.cols, CV_8UC1 );
    float mindisp = 9999.0f;
    float maxdisp = 0.0f;

    for( int i=0; i<disprender.rows; ++i )
    {
        for( int j=0; j<disprender.cols; ++j )
        {
            /*
            float dval = ((float)disp.at< signed short >(i,j))/16.0f;
            if( dval <= 0.0 || dval > numberOfDisparities )
                continue;

            dval -= disp_offset;

            if( j-dval < 0 )
                continue;

            disprender.at<unsigned char>(i,j) = (unsigned char)( (dval/(float)numberOfDisparities)*255.0f );
            */
            mindisp = std::min<float>( mindisp, disp.at< signed short >(i,j));
            maxdisp = std::max<float>( maxdisp, disp.at< signed short >(i,j));

            disprender.at<unsigned char>(i,j) = (unsigned char)( (float)disp.at< signed short >(i,j)/(float)numberOfDisparities*255.0f );
        }
    }
    LOGI << "min/max disp: " << mindisp << " , " << maxdisp << std::endl;

    cv::imwrite( filename, disprender );
}


inline void render_disparity16( std::string filename, const int numberOfDisparities, const int minDisparity, const int disp_offset, const cv::Mat& disp)
{
    cv::Mat disprender = cv::Mat::zeros( disp.rows, disp.cols, CV_8UC1 );
    float mindisp = 9999.0f;
    float maxdisp = 0.0f;

    for( int i=0; i<disprender.rows; ++i )
    {
        for( int j=0; j<disprender.cols; ++j )
        {

            float dval = ((float)disp.at< signed short >(i,j))/16.0f - (float)minDisparity;
            if( dval <= 0.0 || dval > numberOfDisparities )
                continue;

            dval += disp_offset;

            //if( j-dval < 0 )
            //    continue;

            disprender.at<unsigned char>(i,j) = (unsigned char)( (dval/(float)numberOfDisparities)*255.0f );

            mindisp = std::min<float>( mindisp, disp.at< signed short >(i,j));
            maxdisp = std::max<float>( maxdisp, disp.at< signed short >(i,j));

        }
    }
    LOGI << "min/max disp: " << mindisp << " , " << maxdisp << std::endl;

    cv::imwrite( filename, disprender );
}



inline void render_disparity_float( const std::string filename, const cv::Mat& disp )
{
    cv::Mat disprender = cv::Mat::zeros( disp.rows, disp.cols, CV_8UC1 );
    float mindisp = static_cast<float>( disp.cols+1 );
    float maxdisp = 0.0f;

    for( size_t i=0; i<disp.rows; ++i )
    {
        float* pSrc = (float*)disp.ptr((int)i);
        for( size_t j=0; j<disp.cols; ++j )
        {
            mindisp = std::min( *pSrc, mindisp );
            maxdisp = std::max( *pSrc, maxdisp );
            pSrc++;
        }
    }
    for( size_t i=0; i<disp.rows; ++i )
    {
        unsigned char* pDst = (unsigned char*)disprender.ptr((int)i);
        float* pSrc = (float*)disp.ptr((int)i);
        for( size_t j=0; j<disp.cols; ++j )
        {
            *pDst = (unsigned char)( (*pSrc-mindisp)/(maxdisp-mindisp)*255.0f );
            pDst++;pSrc++;
        }
    }

    // Save a scaled version to save some disk space
    cv::Mat disprender_scaled;
    //float scalefac = 600.0f/(float)disprender.rows;
    //cv::resize(disprender,disprender_scaled,cv::Size(),scalefac,scalefac,cv::INTER_LINEAR);
    disprender_scaled = disprender;

    cv::imwrite( filename, disprender_scaled );
}


inline cv::Mat render_stereo( cv::Mat img1, cv::Mat img2 )
{
    cv::Mat out( img1.rows, img1.cols+img2.cols, img1.type() );

    cv::Mat left_side = out.colRange(0,img1.cols );
    img1.copyTo( left_side );

    cv::Mat right_side = out.colRange( img1.cols, out.cols );
    img2.copyTo( right_side );

    return out;
}


inline cv::Mat render_stereo_vertical( cv::Mat img1, cv::Mat img2 )
{
    cv::Mat out( img1.rows+img2.rows, std::max<int>(img1.cols,img2.cols), img1.type() );

    cv::Mat top_side = out.rowRange(0,img1.rows );
    img1.copyTo( top_side );

    cv::Mat bottom_side = out.rowRange( img1.rows, out.rows );
    img2.copyTo( bottom_side );

    return out;
}


inline void show_image( const cv::Mat& img, const double reduce_factor )
{
    cv::Mat img_resized;
    cv::resize( img, img_resized, cv::Size( (int)(img.cols*reduce_factor), (int)(img.rows*reduce_factor)) );
    cv::imshow("image", img_resized );
    cv::waitKey(0);
}


inline void show_time_stats( const cvlab::HiresTimer& timer )
{
    LOGI << "+----------------------------+-------------------+";
    LOGI << "|   Task                     |   Time (seconds)  |";
    LOGI << "+----------------------------+-------------------+";

    double last_t = 0.0;
    for( std::vector< std::pair<double,std::string> >::const_iterator it = timer.evts_begin(); it!=timer.evts_end(); ++it )
    {
        double t = it->first;
        LOGI << "| " << std::setw(25) << it->second <<  "  |" << std::setw(18) << (t-last_t) << " |";
        last_t = t;
    }
    LOGI << "+----------------------------+-------------------+";
    LOGI << "| " << std::setw(25) << "TOTAL" <<  "  |" << std::setw(18) << (timer.elapsed()) << " |";
    LOGI << "+----------------------------+-------------------+";
}


}
}
#endif
