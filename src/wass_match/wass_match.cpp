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



#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>

#include "wassglobal.hpp"
#include "FeatureSet.h"
#include "GTMatcher.h"
#include "log.hpp"
#include "utils.hpp"
#include "incfg.hpp"
#include "epipolar.h"

static boost::filesystem::path workdir;


INCFG_REQUIRE( int, NUM_FEATURES_PER_IMAGE, 2000, "Maxmum number of image features to extract" )
INCFG_REQUIRE( double, MATCHER_LAMBDA, 0.00001, "Matcher payoff lambda" )
INCFG_REQUIRE( double, MATCHER_POPULATION_THRESHOLD, 0.7, "Matcher population threshold" )
INCFG_REQUIRE( int, MATCHER_MIN_GROUP_SIZE, 5, "Matcher minimum required group size" )
INCFG_REQUIRE( int, MATCHER_MAX_ROUNDS, 20, "Matcher maximum number of rounds to perform" )
INCFG_REQUIRE( double, MATCHER_MAX_EPI_DISTANCE, 0.5, "Max matches epipolar distance" )
INCFG_REQUIRE( bool, MATCHER_SKIP_GT, false, "Skip game-theoretic matcher and use the nearest match only." )


bool save_matches( boost::filesystem::path filename, const WASS::match::MatchList& matches )
{
    std::ofstream ofs( filename.string().c_str() );
    if( ofs.fail() )
    {
        LOGE << "Unable to write " << filename.string();
        return false;
    }
    ofs << matches.size() << std::endl;
    ofs << std::setprecision(15);

    for( WASS::match::MatchList::const_iterator it = matches.begin() ; it != matches.end(); ++it )
    {
        ofs << it->imga_loc[0] << " " << it->imga_loc[1] << " " << it->imgb_loc[0] << " " << it->imgb_loc[1] << std::endl;
    }

    ofs.flush();

    return true;
}


cv::Mat render_matches( cv::Mat img1, cv::Mat img2, const WASS::match::MatchList& matches )
{
    cv::Mat out( img1.rows, img1.cols+img2.cols, img1.type() );

    cv::Mat left_side = out.colRange(0,img1.cols );
    img1.copyTo( left_side );

    cv::Mat right_side = out.colRange( img1.cols, out.cols );
    img2.copyTo( right_side );

    cv::Mat aux;
    cv::cvtColor( out, aux, cv::COLOR_GRAY2RGB );
    out = aux;

    for( size_t i=0; i<matches.size(); ++i )
    {
        const WASS::match::Match& m = matches[i];

        cv::circle( out, cv::Point2f( m.imga_loc[0], m.imga_loc[1]), 2, CV_RGB(100,0,0), 1, cv::LINE_AA );
        cv::circle( out, cv::Point2f( m.imgb_loc[0]+img1.cols, m.imgb_loc[1]), 2, CV_RGB(100,0,0), 1, cv::LINE_AA );
        cv::line( out, cv::Point2f( m.imga_loc[0], m.imga_loc[1]), cv::Point2f( m.imgb_loc[0]+img1.cols, m.imgb_loc[1]), CV_RGB(255,255,0), 1, cv::LINE_AA );
    }

    return out;
}


int main( int argc, char* argv[] )
{
    WASS::exe_name_to_stdout( "wass_match" );
    srand( time( 0 ) );

    if( argc == 1 )
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "wass_match [--genconfig] <config_file> <workdir>" << std::endl << std::endl;
        std::cout << "Not enough arguments, aborting." << std::endl;
        return 0;
    }

    if( argc>1 && std::string("--genconfig").compare(  std::string(argv[1]) ) == 0 )
    {
        WASS::setup_logger();
        LOG_SCOPE("wass_match");
        LOGI << "Generating matcher_config.txt ...";

        std::string cfg = incfg::ConfigOptions::instance().to_config_string();

        std::ofstream ofs( "matcher_config.txt" );
        if( !ofs.is_open() )
        {
            LOGE << "Unable to open matcher_config.txt for write";
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
    workdir = boost::filesystem::path( argv[2] );
    if( !boost::filesystem::exists( workdir) )
    {
        std::cerr << workdir << " does not exists, aborting." << std::endl;
        return -1;
    }

    WASS::setup_logger();
    LOG_SCOPE("wass_match");

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

    try
    {
        LOGI << "Matching " << workdir;

        // Features extraction
        cv::Mat img0 = cv::imread( (workdir/"undistorted"/"00000000.png").string(), cv::IMREAD_GRAYSCALE );
        if( img0.rows==0 || img0.cols==0 )
        {
            LOGE << "Unable to open undistorted/00000000.png";
            return -1;
        }

        std::cout << "[P|10|100]" << std::endl;

        WASS::match::FeatureSet f0;
        f0.detect( img0, INCFG_GET(NUM_FEATURES_PER_IMAGE), WASS::match::SURF_Extractor_params::get_default() );
        cv::Mat img0_f = img0.clone();
        f0.renderToImage( img0_f );
        cv::imwrite( (workdir/"00000000_features.jpg").string(), img0_f );
        img0_f = cv::Mat();


        cv::Mat img1 = cv::imread( (workdir/"undistorted"/"00000001.png").string(), cv::IMREAD_GRAYSCALE );
        if( img1.rows==0 || img1.cols==0 )
        {
            LOGE << "Unable to open undistorted/00000001.png";
            return -1;
        }

        std::cout << "[P|20|100]" << std::endl;

        WASS::match::FeatureSet f1;
        f1.detect( img1, INCFG_GET(NUM_FEATURES_PER_IMAGE), WASS::match::SURF_Extractor_params::get_default() );
        cv::Mat img1_f = img1.clone();
        f1.renderToImage( img1_f );
        cv::imwrite( (workdir/"00000001_features.jpg").string(), img1_f );
        img1_f = cv::Mat();



        // Matching
        WASS::match::GTMatcher matcher( f0, f1 );
        WASS::match::MatchList all_matches;

        LOGI << "generating candidate matches";
        all_matches = matcher.generate_candidates();

        if( !INCFG_GET( MATCHER_SKIP_GT ) )
        {
            bool continue_matching = true;
            int max_rounds = INCFG_GET(MATCHER_MAX_ROUNDS);
            all_matches.clear();
            do
            {
                matcher.compute_payoff_matrix( INCFG_GET(MATCHER_LAMBDA) );
                WASS::match::MatchList curr_group = matcher.match_group( INCFG_GET(MATCHER_POPULATION_THRESHOLD) );
                all_matches.insert( all_matches.end(), curr_group.begin(), curr_group.end() );

                if( curr_group.size() < INCFG_GET(MATCHER_MIN_GROUP_SIZE) )
                    continue_matching = false;

                std::cout << "[P|" << static_cast<int>(static_cast<float>(INCFG_GET(MATCHER_MAX_ROUNDS)-max_rounds)/static_cast<float>(INCFG_GET(MATCHER_MAX_ROUNDS)) * 70.0f + 20.0f) << "|100]" << std::endl;

            } while( max_rounds-- && continue_matching );
        } 
        else 
        {
            LOGI << "skipping GT matcher";
        }

        cv::Mat debug_matches = render_matches( img0, img1, all_matches );
        cv::imwrite( (workdir/"matches.jpg").string(), debug_matches );

        LOGI << "Saving matches";
        if( !save_matches(workdir/"matches_unfiltered.txt", all_matches) )
            return -1;

        LOGI << all_matches.size() << " total matches recovered";

        LOGI << "epipolar filter...";

        cv::Mat K0 = WASS::load_matrix( workdir / "intrinsics_00000000.xml" );
        cv::Mat K1 = WASS::load_matrix( workdir / "intrinsics_00000001.xml" );

        double focal = ( K0.at<double>(0,0)+K0.at<double>(1,1) ) * 0.5;

        K0 = K0.inv();
        K1 = K1.inv();
        cv::Matx33d K0i((double*)(K0.clone().ptr()));
        cv::Matx33d K1i((double*)(K1.clone().ptr()));


        std::vector< cv::Point2d > pts_0;
        std::vector< cv::Point2d > pts_1;

        pts_0.reserve( all_matches.size() );
        pts_1.reserve( all_matches.size() );

        for( size_t i=0; i<all_matches.size(); ++i )
        {
            const WASS::match::Match& m = all_matches[i];
            cv::Vec3d cp0 = K0i * cv::Vec3d( m.imga_loc[0], m.imga_loc[1], 1 );
            cv::Vec3d cp1 = K1i * cv::Vec3d( m.imgb_loc[0], m.imgb_loc[1], 1 );

            pts_0.push_back( cv::Point2d(cp0[0], cp0[1]) );
            pts_1.push_back( cv::Point2d(cp1[0], cp1[1]) );
        }

        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat( pts_0, pts_1, 1.0, cv::Point2d(0,0), cv::RANSAC, 0.9999, INCFG_GET(MATCHER_MAX_EPI_DISTANCE)/focal, mask );

        size_t num_inliers=0;
        for( int i=0; i<mask.rows*mask.cols; ++i)
            num_inliers += mask.at<bool>( i )?1:0;

        LOGI << num_inliers << " inliers after epipolar filter (max epi distance allowed: " << INCFG_GET(MATCHER_MAX_EPI_DISTANCE) << " px)" << std::endl;

        LOGI << "computed essential matrix: " << std::endl << E;
        {
            WASS::match::MatchList all_matches_filtered;
            for( size_t i=0; i<all_matches.size(); ++i )
            {
                if( mask.at<bool>( static_cast<int>(i) ) )
                {
                    all_matches_filtered.push_back( all_matches[i] );
                }
            }
            LOGI << "Saving matches";
            if( !save_matches(workdir/"matches_epionly.txt", all_matches_filtered) )
                return -1;
        }


        LOGI << "recovering pose...";

        cv::Mat R;
        cv::Mat T;
        cv::recoverPose( E, pts_0, pts_1, R, T, 1.0, cv::Point2d(0,0), mask );

        LOGI << "relative rotation: " << std::endl << R;
        LOGI << "relative translation: " << std::endl << T;

        // Render inliers
        std::vector< cv::Vec2d > pts0_px;
        std::vector< cv::Vec2d > pts1_px;

        WASS::match::MatchList all_matches_filtered;
        for( size_t i=0; i<all_matches.size(); ++i )
        {
            if( mask.at<bool>( static_cast<int>(i) ) )
            {
                all_matches_filtered.push_back( all_matches[i] );
                pts0_px.push_back( (all_matches[i]).imga_loc );
                pts1_px.push_back( (all_matches[i]).imgb_loc );
            }
        }

        LOGI << all_matches_filtered.size() << " matches have passed the chirality check";
        debug_matches = render_matches( img0, img1, all_matches_filtered );
        cv::imwrite( (workdir/"matches_epifilter.jpg").string(), debug_matches );


        cv::Matx33d Ex((double*)(E.clone().ptr()));
        cv::Matx33d F = K1i.t() * Ex * K0i;
        WASS::epi::ErrorStats er = WASS::epi::evaluate_epipolar_error( F, pts0_px, pts1_px );
        LOGI << "Epipolar error: " << er.avg <<  "+-" << er.std << " px. Min: " << er.min << " Max: " << er.max;

        {
            // Save error stats
            std::ofstream ofs( (workdir/"matcher_stats.csv").c_str() );
            ofs << std::setprecision(15);
            ofs << "N.Matches;Avg. Error;Std. Error;Min. Error;Max. Error" << std::endl;
            ofs << all_matches_filtered.size() << ";" << er.avg << ";" << er.std << ";" << er.min << ";" << er.max << std::endl;
            ofs.close();
        }

        cv::FileStorage fs( (workdir/"ext_R.xml").string(), cv::FileStorage::WRITE );
        fs << "ext_R" << R;
        fs.release();
        fs.open( (workdir/"ext_T.xml").string(), cv::FileStorage::WRITE );
        fs << "ext_T" << T;
        fs.release();


        LOGI << "Saving matches";

        if( !save_matches(workdir/"matches.txt", all_matches_filtered ) )
            return -1;

        LOGI << "All done!";
        std::cout << "[P|100|100]" << std::endl;

    } catch( WASS::match::FeatureSet::FeatureExtractorException& ex )
    {
        LOGE << ex.what();
        return -1;

    } catch( WASS::match::GTMatcher::GTMatcherException& ex )
    {
        LOGE << ex.what();
        return -1;
    }

    return 0;
}
