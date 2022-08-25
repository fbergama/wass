#ifndef _WASS_POINT_PICKER_HPP_
#define _WASS_POINT_PICKER_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>



struct MouseCallbackHandlerData
{
	cv::Point2d p1;
	cv::Point2d p2;
	cv::Size framesize;
	double display_scale;
	cv::Point2d traslc;
        cv::Point2d point_loc;
        bool point_loc_set;
        bool needs_repaint;
};


void mouse_callback_handler(int event, int x, int y, int flags, void *userdata)
{
    MouseCallbackHandlerData* mdata = reinterpret_cast<MouseCallbackHandlerData*>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN )
    {
        mdata->traslc = cv::Point2i(x, y);
        return;
    }
    if (event == cv::EVENT_MOUSEMOVE && ( flags & cv::EVENT_FLAG_LBUTTON ) )
    {
        cv::Point2i delta = mdata->traslc - cv::Point2d(x, y);

        if ( (mdata->p1.x + delta.x) >= 0 && (mdata->p2.x + delta.x) <  mdata->framesize.width )
        {
            mdata->p1.x += delta.x;
            mdata->p2.x += delta.x;
        }
        if ((mdata->p1.y + delta.y) >= 0 && (mdata->p2.y + delta.y) <  mdata->framesize.height)
        {
            mdata->p1.y += delta.y;
            mdata->p2.y += delta.y;
        }
        mdata->traslc = cv::Point2d(x, y);
        mdata->needs_repaint = true;

        return;
    }


    int wheel = cv::getMouseWheelDelta(flags);

    if (event == cv::EVENT_MOUSEMOVE && ( flags & cv::EVENT_FLAG_CTRLKEY ) )
    {
         wheel = y-mdata->traslc.y;
         mdata->traslc = cv::Point2i(x, y);
    }

    if ( wheel!=0 )
    {
        double scale = wheel>0?1.1:0.9;
        if (scale < 1.0 && (mdata->p2.x - mdata->p1.x) < 10)
            return;

        double currwsizeW = mdata->p2.x - mdata->p1.x;
        double currwsizeH = mdata->p2.y - mdata->p1.y;
        cv::Point2d zoom_center(mdata->p1.x + x * mdata->display_scale / mdata->framesize.width * currwsizeW,
                mdata->p1.y + y * mdata->display_scale / mdata->framesize.width * currwsizeH);

        mdata->p1 = (mdata->p1 - zoom_center)*scale + zoom_center;
        mdata->p2 = (mdata->p2 - zoom_center)*scale + zoom_center;


        if ( (mdata->p2.x - mdata->p1.x) >= mdata->framesize.width-1 || (mdata->p2.y - mdata->p1.y) >= mdata->framesize.height - 1)
        {
            mdata->p1 = cv::Point2i(0, 0);
            mdata->p2 = cv::Point2i(mdata->framesize.width-1, mdata->framesize.height-1);
            return;
        }

        if (mdata->p1.x < 0)
        {
            mdata->p1.x -= mdata->p1.x;
            mdata->p2.x -= mdata->p1.x;
        }
        if (mdata->p1.y < 0)
        {
            mdata->p1.y -= mdata->p1.y;
            mdata->p2.y -= mdata->p1.y;
        }
        if (mdata->p2.x >= mdata->framesize.width)
        {
            mdata->p1.x -= mdata->p2.x - mdata->framesize.width+1;
            mdata->p2.x -= mdata->p2.x - mdata->framesize.width+1;
        }
        if (mdata->p2.y >= mdata->framesize.height)
        {
            mdata->p1.y -= mdata->p2.y - mdata->framesize.height+1;
            mdata->p2.y -= mdata->p2.y - mdata->framesize.height+1;
        }
        mdata->needs_repaint = true;

        return;
    }

    if ( event == cv::EVENT_RBUTTONDOWN )
    {
        double xratio = static_cast<double>(x)/(mdata->framesize.width / mdata->display_scale);
        double yratio = static_cast<double>(y)/(mdata->framesize.height / mdata->display_scale);

        mdata->point_loc.x = mdata->p1.x + xratio*(mdata->p2.x-mdata->p1.x);
        mdata->point_loc.y = mdata->p1.y + yratio*(mdata->p2.y-mdata->p1.y);
        mdata->point_loc_set = true;
        mdata->needs_repaint = true;

        return;
    }
}

class PointPicker
{
public:

    inline PointPicker( std::string winname, const cv::Mat& img ) : wname( winname ), cvimg( img )
    {
        mousedata.p1 = cv::Point(0, 0);
        mousedata.p2 = cv::Point(cvimg.cols, cvimg.rows);
        mousedata.display_scale = 3.0;
        mousedata.framesize = cv::Size(cvimg.cols, cvimg.rows);
        mousedata.needs_repaint = true;
        mousedata.point_loc_set = false;

        cv::namedWindow( wname, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL ); // The function does nothing if the window already exists
        cv::moveWindow( wname, 0, 0);
        //cv::imshow( wname, img );
        cv::setMouseCallback( wname, mouse_callback_handler, &(this->mousedata) );

        std::cout << "Right click to set point, ENTER to accept. Ctrl+mouse move to zoom" << std::endl;
    }


    inline void loop()
    {
        cv::Mat iclone = cvimg.clone();
        cv::Mat displayedimg;

        while(1)
        {

            if( mousedata.point_loc_set )
            {
                iclone = cvimg.clone();
                cv::drawMarker( iclone, mousedata.point_loc, CV_RGB(0,0,255), 0, cvimg.rows/100, 1 );
                mousedata.point_loc_set = false;
            }

            if( mousedata.needs_repaint )
            {
                cv::resize( iclone( cv::Rect(mousedata.p1, mousedata.p2) ), displayedimg, cv::Size(cvimg.cols / mousedata.display_scale, cvimg.rows / mousedata.display_scale) , 0, 0, cv::INTER_NEAREST);

                mousedata.needs_repaint = false;
            }

            cv::imshow( wname, displayedimg );

            int k = cv::waitKey(20);
            if( k==13 || k==10 )
                break;
        }
    }
    ~PointPicker()
    {
        cv::destroyWindow( wname );
    }

    inline cv::Point2d selected_point() { return mousedata.point_loc; }

private:
    std::string wname;
    const cv::Mat& cvimg;
    MouseCallbackHandlerData mousedata;


};

#endif
