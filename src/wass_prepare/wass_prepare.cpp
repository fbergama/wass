#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "wassglobal.hpp"
#include "log.hpp"
#include "utils.hpp"


namespace po = boost::program_options;



int main( int argc, char* argv[] )
{
    WASS::exe_name_to_stdout( "wass_prepare" );
    po::options_description desc("wass_prepare arguments");
    desc.add_options()
        ("help", "produce help message")
        ("workdir", po::value<std::string>(), "Workdir name")
        ("calibdir", po::value<std::string>(), "Directory containing calibration data")
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
    po::store( po::parse_command_line( argc, argv, desc), vm );
    po::notify( vm );


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
    if( !boost::filesystem::create_directories( wdir ) )
    {
        LOGE << "Unable to create.";
        return -1;
    }

    cv::Mat intr0;
    cv::Mat dist0;
    cv::Mat intr1;
    cv::Mat dist1;

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

    img = cv::imread( vm["c1"].as<std::string>(), cv::IMREAD_GRAYSCALE );
    img_undist;
    cv::undistort( img, img_undist, intr1, dist1 );
    cv::imwrite( (undist_dir/"00000001.png").string(), img_undist );


    cv::Mat extR = WASS::load_matrix( calibdir/"ext_R.xml" );
    cv::Mat extT = WASS::load_matrix( calibdir/"ext_T.xml" );
    if( extR.rows ==3 && extR.cols==3 && extT.rows == 3 && extT.cols == 3 )
    {
        LOGI << "Extrinsic calibration found";
    }
    else
    {
        LOGE << "Extrinsic calibration not found.";
    }


    LOGI << "Saving calibration data";

    cv::FileStorage fs( (wdir/"intrinsics_00000000.xml").string(), cv::FileStorage::WRITE );
    fs << "intr" << intr0;
    fs.release();
    fs.open( (wdir/"intrinsics_00000001.xml").string(), cv::FileStorage::WRITE );
    fs << "intr" << intr1;
    fs.release();


    LOGI << "All done, exiting";

    return 0;
}
