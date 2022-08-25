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


#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "wassglobal.hpp"
#include "log.hpp"
#include "utils.hpp"


namespace po = boost::program_options;



int main( int argc, char* argv[] )
{
    WASS::exe_name_to_stdout( "wass_prepare" );
    po::options_description desc("wass_prepare arguments");
    desc.add_options()
        ("workdir", po::value<std::string>(), "Workdir name")
        ("calibdir", po::value<std::string>(), "Calibration data directory")
        ("c0", po::value<std::string>(), "Cam0 image file")
        ("c1", po::value<std::string>(), "Cam1 image file")
    ;

    WASS::setup_logger();
    LOG_SCOPE("wass_prepare");

    if( argc==1 )
    {
        std::cout << desc << std::endl;
        return 0;
    }

    po::variables_map vm;

    try
    {
        po::store( po::parse_command_line( argc, argv, desc), vm );
        po::notify( vm );

    } catch( boost::program_options::unknown_option& ex )
    {
        LOGE << ex.what();
        return -1;

    } catch( std::exception& ex2 )
    {
        LOGE << ex2.what();
        return -1;

    } catch( ... )
    {
        LOGE << "Generic program options parse error";
        return -1;
    }


    if( !vm.count("workdir") )
    {
        LOGE << "workdir option not specified";
        return -1;
    }

    if( !vm.count("calibdir") )
    {
        LOGE << "calibdir option not specified";
        return -1;
    }

    if( !vm.count("c0") || !vm.count("c1") )
    {
        LOGE << "c0 and c1 options must be both specified";
        return -1;
    }

    boost::filesystem::path calibdir( vm["calibdir"].as<std::string>() );
    if( !boost::filesystem::exists(calibdir) || !boost::filesystem::is_directory( calibdir ) )
    {
        LOGE << "Invalid calibration directory";
        return -1;
    }


    boost::filesystem::path wdir( vm["workdir"].as<std::string>() );

    if( boost::filesystem::exists( wdir ) )
    {
        LOGE << absolute(wdir) << " already exists.";
        return -1;
    }

    LOGI << "Creating " << wdir;
    boost::filesystem::create_directories( wdir );

    cv::Mat intr0;
    cv::Mat dist0;
    cv::Mat intr1;
    cv::Mat dist1;

    std::cout << "[P|10|100]" << std::endl;
    LOGI << "Loading calibration data";

    if( (intr0 = WASS::load_matrix( calibdir/"intrinsics_00.xml") ).rows == 0 )
        return -1;

    if( (dist0 = WASS::load_matrix( calibdir/"distortion_00.xml") ).rows == 0 )
    {
        LOGI << (calibdir/"distortion_00.xml") << " not found. Assuming no distortion.";
        dist0 = cv::Mat::zeros(5,1,CV_64FC1);
    }

    if( (intr1 = WASS::load_matrix( calibdir/"intrinsics_01.xml") ).rows == 0 )
        return -1;

    if( (dist1 = WASS::load_matrix( calibdir/"distortion_01.xml") ).rows == 0 )
    {
        LOGI << (calibdir/"distortion_01.xml") << " not found. Assuming no distortion.";
        dist1 = cv::Mat::zeros(5,1,CV_64FC1);
    }


    std::cout << "[P|20|100]" << std::endl;
    LOGI << "Undistorting images";

    boost::filesystem::path undist_dir( wdir/"undistorted" );
    if( !boost::filesystem::create_directories( undist_dir ) )
    {
        LOGE << "Unable to create " << undist_dir;
        return -1;
    }

    cv::Mat img = cv::imread( vm["c0"].as<std::string>(), cv::IMREAD_GRAYSCALE );
    cv::Mat img_undist;
    cv::undistort( img, img_undist, intr0, dist0 );
    cv::imwrite( (undist_dir/"00000000.png").string(), img_undist );

    std::cout << "[P|50|100]" << std::endl;

    img = cv::imread( vm["c1"].as<std::string>(), cv::IMREAD_GRAYSCALE );
    cv::undistort( img, img_undist, intr1, dist1 );
    cv::imwrite( (undist_dir/"00000001.png").string(), img_undist );

    std::cout << "[P|70|100]" << std::endl;

    cv::Mat extR = WASS::load_matrix( calibdir/"ext_R.xml" );
    cv::Mat extT = WASS::load_matrix( calibdir/"ext_T.xml" );
    if( extR.rows ==3 && extR.cols==3 && extT.rows == 3 && extT.cols == 1 )
    {
        LOGI << "Extrinsic calibration found, copying to destination workdir";
        {
            cv::FileStorage fs( (wdir/"ext_R.xml").string(), cv::FileStorage::WRITE );
            fs << "R" << extR;
            fs.release();
        }
        {
            cv::FileStorage fs( (wdir/"ext_T.xml").string(), cv::FileStorage::WRITE );
            fs << "T" << extT;
            fs.release();
        }
    }
    else
    {
        LOGE << "Extrinsic calibration not found.";
    }


    LOGI << "Saving intrinsic calibration data";

    cv::FileStorage fs( (wdir/"intrinsics_00000000.xml").string(), cv::FileStorage::WRITE );
    fs << "intr" << intr0;
    fs.release();
    fs.open( (wdir/"intrinsics_00000001.xml").string(), cv::FileStorage::WRITE );
    fs << "intr" << intr1;
    fs.release();


    LOGI << "All done, exiting";
    std::cout << "[P|100|100]" << std::endl;

    return 0;
}
