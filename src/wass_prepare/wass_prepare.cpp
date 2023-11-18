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


#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "wassglobal.hpp"
#include "log.hpp"
#include "utils.hpp"
#include "incfg.hpp"


INCFG_REQUIRE( double, CAM0_CLAHE_CLIPLIMIT, 2.0, "CAM0 CLAHE cliplimit parameter" )
INCFG_REQUIRE( int, CAM0_CLAHE_TILEGRIDSIZE, 0, "CAM0 CLAHE tile grid size (set to 0 to disable CLAHE). 150 is a good value to start" )
INCFG_REQUIRE( double, CAM1_CLAHE_CLIPLIMIT, 2.0, "CAM1 CLAHE cliplimit parameter" )
INCFG_REQUIRE( int, CAM1_CLAHE_TILEGRIDSIZE, 0, "CAM1 CLAHE tile grid size (set to 0 to disable CLAHE). 150 is a good value to start" )


namespace po = boost::program_options;


/*
 * Demosaics an image I assuming the following macropixel ordering:
 *
 *  out[2]  out[1]
 *  out[3]  out[0]
 *
 */
void demosaic( const cv::Mat& I, cv::Mat out[4] )
{
    /*
     * I0 = I[1::2,1::2]
     * I45 = I[::2,1::2]
     * I90 = I[::2,::2]
     * I135 = I[1::2,::2]
    */
    const int m = I.rows / 2;
    const int n = I.cols / 2;

    for( int i=0; i<4; ++i )
        out[i] = cv::Mat::zeros( m, n, CV_8UC1 );

    for( int i=0; i<m; ++i )
    {
        const unsigned char* pI0 = I.ptr( 2*i );
        const unsigned char* pI1 = I.ptr( 2*i+1 );

        unsigned char* pA = out[0].ptr( i );
        unsigned char* pB = out[1].ptr( i );
        unsigned char* pC = out[2].ptr( i );
        unsigned char* pD = out[3].ptr( i );
        for( int j=0; j<n; ++j )
        {
            *pC++ = *pI0;
            *pB++ = *(pI0+1);
            *pA++ = *(pI1+1);
            *pD++ = *pI1;
            pI0 += 2;
            pI1 += 2;
        }
    }
}


void process_image( std::string filename, const cv::Mat K, const cv::Mat dist, cv::Ptr< cv::CLAHE > clahe, boost::filesystem::path outdir, std::string outfile,
                    bool do_demosaic )
{
    LOG_SCOPE("wass_prepare");
    cv::Mat img = cv::imread( filename, cv::IMREAD_GRAYSCALE );
    cv::Mat img_undist;

    if( img.rows == 0 )
    {
        LOGE << "Unable to load" << filename;
        return;
    }
    //cv::imwrite( (outdir/"original.png").string(), img );
    LOGI << "Image size: " << img.cols << "x" << img.rows;

    if( do_demosaic )
    {
        LOGI << "Demosaicing...";
        cv::Mat ch[] = {cv::Mat(),cv::Mat(),cv::Mat(),cv::Mat() };
        cv::Mat& I0=ch[0];
        cv::Mat& I45=ch[1];
        cv::Mat& I90=ch[2];
        cv::Mat& I135=ch[3];
        cv::Mat aux;
        demosaic( img, ch );

        // Upscale using BiCubic interpolation
        for( int i=0; i<4; ++i )
        {
            cv::Mat aux2;
            cv::resize( ch[i], aux, cv::Size(), 2, 2, cv::INTER_CUBIC ); 
            cv::undistort( aux, aux2, K, dist );
            aux2.convertTo(ch[i],CV_32F,1.0f/255.0f);
        }


        // HDR, ref: 
        // Wu, X., Zhang, H., Hu, X., Shakeri, M., Fan, C., Ting, J.
        // Hdr reconstruction based on the polarization camera. IEEE Robotics and Automation Letters 5(4),
        // 5113–5119 (2020)
        //
        float sig=0.35f;
        cv::Mat w0 = -1.0f*(I0-0.5f).mul(I0-0.5f)/(2.0f*sig*sig); 
        cv::exp(w0,w0);
        cv::Mat w45 = -1.0f*(I45-0.5f).mul(I45-0.5f)/(2.0f*sig*sig); 
        cv::exp(w45,w45);
        cv::Mat w90 = -1.0f*(I90-0.5f).mul(I90-0.5f)/(2.0f*sig*sig); 
        cv::exp(w90,w90);
        cv::Mat w135 = -1.0f*(I135-0.5f).mul(I135-0.5f)/(2.0f*sig*sig); 
        cv::exp(w135,w135);


        cv::Mat HDR = (w0.mul(I0) + w45.mul(I45) + w90.mul(I90) + w135.mul(I135))/(w0+w45+w90+w135);
        HDR.convertTo(img,CV_8UC1,255.0f);


        // Stokes
        cv::Mat S0 = (I0+I45+I90+I135)/4.0;
        cv::Mat S1 = I0 - I90;
        cv::Mat S2 = I45 - I135;

        cv::Mat dolp = S1.mul(S1) + S2.mul(S2);
        cv::sqrt(dolp,dolp);
        dolp = dolp / S0;

        cv::Mat dolp_color;
        dolp.convertTo(aux,CV_8UC1,255.0);
        cv::applyColorMap(aux, dolp_color, cv::COLORMAP_JET);
        cv::imwrite( (outdir/"dolp.jpg").string(), dolp_color );

        cv::Mat aolp;
        cv::cartToPolar( S2, S1, aux, aolp, false );
        aolp = (aolp-3.1415) * 0.5;

        cv::Mat aolp_color;
        aolp.convertTo(aux, CV_8UC1, 255.0/3.1415, 127.0);
        cv::applyColorMap(aux, aolp_color, cv::COLORMAP_JET);
        cv::imwrite( (outdir/"aolp.jpg").string(), aolp_color );

        
        I0.convertTo( aux, CV_8UC1, 255.0f );
        cv::imwrite( (outdir/"I0.png").string(), aux );
        I45.convertTo( aux, CV_8UC1, 255.0f );
        cv::imwrite( (outdir/"I45.png").string(), aux );
        I90.convertTo( aux, CV_8UC1, 255.0f );
        cv::imwrite( (outdir/"I90.png").string(), aux );
        I135.convertTo( aux, CV_8UC1, 255.0f );
        cv::imwrite( (outdir/"I135.png").string(), aux );

    }

    if( clahe )
    {
        cv::Mat dst;
        clahe->apply( img, dst );
        img = dst;
    }

    if( !do_demosaic )
    {
        // Demosaic process already undistort the images. So this line
        // is needed only if do_demosaic is false
        cv::undistort( img, img_undist, K, dist );
    } else
    {
        img_undist = img;
    }


    cv::imwrite( (outdir/outfile).string(), img_undist );
    LOGI << "Output image size: " << img_undist.cols << "x" << img_undist.rows;

    return;
}


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
    WASS::exe_name_to_stdout( "wass_prepare" );
    po::options_description desc("wass_prepare arguments");
    desc.add_options()
        ("workdir", po::value<std::string>(), "Workdir name")
        ("calibdir", po::value<std::string>(), "Calibration data directory")
        ("c0", po::value<std::string>(), "Cam0 image file")
        ("c1", po::value<std::string>(), "Cam1 image file")
        ("demosaic", po::bool_switch()->default_value(false), "Demosaic polarimetric images")
        ("continue-if-existing", po::bool_switch()->default_value(false), "Don't complain if output dir already exists")
        ("genconfig", po::bool_switch()->default_value(false), "Generate configuration file")
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
    
    if( vm["genconfig"].as<bool>() )
    {
        save_configuration("prepare_config.txt");
        return 0;
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

    if( !vm["continue-if-existing"].as<bool>() &&  boost::filesystem::exists( wdir ) )
    {
        LOGE << absolute(wdir) << " already exists.";
        return -1;
    }



    try
    {
        LOGI << "Checking if configuration file exists...";
        std::string config_filename =  (calibdir/"prepare_config.txt").string();

        std::ifstream ifs( config_filename );
        if( !ifs.is_open() )
        {
            LOGE << "Unable to load " << config_filename;
        }
        else
        {
            incfg::ConfigOptions::instance().load( ifs );
            LOGI << "Settings loaded";
        }

    } catch( std::runtime_error& er )
    {
        LOGE << er.what();
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
    LOGI << "Processing images";

    boost::filesystem::path undist_dir( wdir/"undistorted" );
    if( !boost::filesystem::create_directories( undist_dir ) && !vm["continue-if-existing"].as<bool>() )
    {
        LOGE << "Unable to create " << undist_dir;
        return -1;
    }

    cv::Ptr< cv::CLAHE > clahe;
    if( INCFG_GET( CAM0_CLAHE_TILEGRIDSIZE ) > 0 )
    {
        clahe = cv::createCLAHE( INCFG_GET( CAM0_CLAHE_CLIPLIMIT ), cv::Size( INCFG_GET(CAM0_CLAHE_TILEGRIDSIZE), INCFG_GET(CAM0_CLAHE_TILEGRIDSIZE) ) );
    }

#if 0
    cv::Mat img = cv::imread( vm["c0"].as<std::string>(), cv::IMREAD_GRAYSCALE );
    if( clahe )
    {
        cv::Mat dst;
        clahe->apply( img, dst );
        img = dst;
    }

    cv::Mat img_undist;
    cv::undistort( img, img_undist, intr0, dist0 );
    cv::imwrite( (undist_dir/"00000000.png").string(), img_undist );
#endif
    process_image( vm["c0"].as<std::string>(), intr0, dist0, clahe, undist_dir, "00000000.png", vm["demosaic"].as<bool>() );

    std::cout << "[P|50|100]" << std::endl;

    clahe.release();
    if( INCFG_GET( CAM1_CLAHE_TILEGRIDSIZE ) > 0 )
    {
        clahe = cv::createCLAHE( INCFG_GET( CAM1_CLAHE_CLIPLIMIT ), cv::Size( INCFG_GET(CAM1_CLAHE_TILEGRIDSIZE), INCFG_GET(CAM1_CLAHE_TILEGRIDSIZE) ) );
    }

#if 0
    img = cv::imread( vm["c1"].as<std::string>(), cv::IMREAD_GRAYSCALE );
    if( clahe )
    {
        cv::Mat dst;
        clahe->apply( img, dst );
        img = dst;
    }
    cv::undistort( img, img_undist, intr1, dist1 );
    cv::imwrite( (undist_dir/"00000001.png").string(), img_undist );
#endif
    process_image( vm["c1"].as<std::string>(), intr1, dist1, clahe, undist_dir, "00000001.png", vm["demosaic"].as<bool>() );

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
