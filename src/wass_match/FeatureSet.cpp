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

#include <opencv2/core/types.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include "FeatureSet.h"
#include "hires_timer.h"
#include "log.hpp"
#include "incfg.hpp"
#include <fstream>
#include <boost/cstdint.hpp>
#include <boost/scoped_array.hpp>

#include <opencv2/flann/flann.hpp>

#include <opencv2/features2d.hpp>


using WASS::match::FeatureSet;
using WASS::match::Feature;
using WASS::match::SURF_Extractor_params;


INCFG_REQUIRE( double, FEATURE_MIN_DISTANCE, 10.0, "Minimum distance allowed between two features (in px)" )
INCFG_REQUIRE( float, FEATURE_HESSIAN_THRESHOLD, 0.0001f, "OpenSURF Hessian threshold" )
INCFG_REQUIRE( int, FEATURE_N_OCTAVES, 4, "OpenSURF number of octaves" )
INCFG_REQUIRE( int, FEATURE_N_LAYERS, 4, "OpenSURF number of layers" )
INCFG_REQUIRE( int, FEATURE_INIT_SAMPLES, 1, "OpenSURF init samples" )
INCFG_REQUIRE( int, AREA_SUBDIVISION, 5, "Number of image subdivisons to improve feature distribution" )


/****************************************
 * Utilities
 ****************************************/

inline double dist2d( const cv::KeyPoint& a, const cv::KeyPoint& b )
{
    return sqrt( (a.pt.x-b.pt.x)*(a.pt.x-b.pt.x) + (a.pt.y-b.pt.y)*(a.pt.y-b.pt.y) );
}

typedef std::pair< cv::KeyPoint, int > keypoint_index_pair;

struct ImageArea {
    cv::Rect area;
    std::vector< keypoint_index_pair > surfs;
};

bool sort_by_response( const keypoint_index_pair& a, const keypoint_index_pair& b ) {
    return (a.first.response > b.first.response);
}

void create_areas( const int img_width, const int img_height, const int num_subdivisions, const int border_width, const std::vector< cv::KeyPoint >& detected_surfs, std::vector< ImageArea >& areas ) {

    const float width = (float)img_width/(float)num_subdivisions;
    const float height = (float)img_height/(float)num_subdivisions;
    for(  int ii=0; ii<num_subdivisions; ii++ ) {
        for(  int jj=0; jj<num_subdivisions; jj++ ) {
            ImageArea currArea;
            currArea.area = cv::Rect( static_cast<int>( (float)img_width/(float)num_subdivisions*(float)ii), static_cast<int>( (float)img_height/(float)num_subdivisions*(float)jj), (int)width, (int)height ) ;
            areas.push_back( currArea );
        }
    }
    for( unsigned int i=0; i<detected_surfs.size(); i++ ) {
        for( unsigned int j=0; j<areas.size(); j++ ) {
            if( detected_surfs[i].pt.x > border_width &&
                detected_surfs[i].pt.x < img_width-border_width &&
                detected_surfs[i].pt.y > border_width &&
                detected_surfs[i].pt.y < img_height-border_width &&
                detected_surfs[i].pt.x > areas[j].area.x &&
                detected_surfs[i].pt.y > areas[j].area.y &&
                detected_surfs[i].pt.x < areas[j].area.x + areas[j].area.width &&
                detected_surfs[i].pt.y < areas[j].area.y + areas[j].area.height  ) {

                    areas[j].surfs.push_back( keypoint_index_pair( detected_surfs[i], i ) );
            }
        }

    }
}






/****************************************
 * FeatureSet
 ****************************************/

SURF_Extractor_params SURF_Extractor_params::get_default()
{
    SURF_Extractor_params prm;
    prm.n_octaves=INCFG_GET(FEATURE_N_OCTAVES);
    prm.n_octave_layers=INCFG_GET(FEATURE_N_LAYERS);
    prm.init_samples=INCFG_GET(FEATURE_INIT_SAMPLES);
    prm.hessian_thresh = INCFG_GET(FEATURE_HESSIAN_THRESHOLD);
    return prm;
}


namespace WASS {
namespace match {

    class FeatureSetImpl
    {
    public:
        FeatureSetImpl() : indexParams(5),kdtree(0) {}

        std::vector< Feature > fts;
        std::vector< ImageArea > areas;

        // KDTREE
        cv::Mat kdt_features;
        int n_dimensions;
        cv::flann::KDTreeIndexParams indexParams;
        cv::flann::Index* kdtree;
    };
}
}


FeatureSet::FeatureSet()
{
    pImpl = new FeatureSetImpl();
}


FeatureSet::~FeatureSet()
{
    clear();
    delete pImpl;
    pImpl = 0;
}


Feature& FeatureSet::operator[]( size_t idx ) const
{
    return pImpl->fts[idx];

}


size_t FeatureSet::size() const
{
    return pImpl->fts.size();

}


void FeatureSet::clear()
{
    clear_kdtree();
    pImpl->fts.clear();
}



void FeatureSet::detect( cv::Mat img, size_t max_features, SURF_Extractor_params prms )
{
    LOG_SCOPE("FeatureSet");
    clear();

    cvlab::HiresTimer timer;
    timer.start();

    // First we ensure that the image is in the correct format
    if( img.channels() != 1 )
    {
        throw FeatureExtractorException("Only 1-channel images accepted");
    }

    cv::Mat surf_input = img;
    if( img.depth() != CV_8UC1 )
    {
        // The image is probably floating point ranging from 0.0f to 1.0f
        img.convertTo( surf_input, CV_8UC1, 255.0 );
    }


    //auto feature_detector = cv::SIFT::create( 10000, prms.n_octaves );
    auto feature_detector = cv::KAZE::create( false, false, prms.hessian_thresh, prms.n_octaves, prms.n_octave_layers );

    LOGI << "extracting " << feature_detector->getDefaultName() << " features";
    LOGI << prms;

    std::vector< cv::KeyPoint > ipts;
    cv::Mat descriptors;
    feature_detector->detectAndCompute( surf_input, cv::noArray(), ipts, descriptors );

    LOGI << ipts.size()  <<  " features found.";
    //LOGI << " descriptors shape: " << descriptors.rows << " x " << descriptors.cols ;
    //LOGI << " descriptors type:" << descriptors.type();

    if( descriptors.type() != CV_32F )
    {
        LOGE << "Invalid descriptor type: float required";
        return;
    }

    LOGI << "Subsampling...";

    if( ipts.size() > 0 )
    {
        std::vector< ImageArea >& areas = pImpl->areas;

        // Trim features by area
        create_areas( surf_input.cols, surf_input.rows, INCFG_GET(AREA_SUBDIVISION), std::max<int>( (int)(surf_input.cols/30.0), 2 ), ipts, areas );

        int points_per_area = static_cast<int>( max_features / areas.size() );
        int num_extra_pts_available = 0;
        for( unsigned int i=0; i<areas.size(); i++ ) {
            if( areas[i].surfs.size() < points_per_area ) {
                num_extra_pts_available += static_cast<int>( points_per_area - areas[i].surfs.size() );
            }
        }
        points_per_area = (int)( (float)points_per_area + (float)num_extra_pts_available/(float)areas.size() );
        LOGI << points_per_area << " points per area";

        ipts.clear();
        pImpl->fts.clear();

        for( unsigned int i=0; i<areas.size(); i++ ) {
            //LOGI << "Area " << i << ": " << areas[i].surfs.size() << " points.";

            if( areas[i].surfs.size() < 2 )
                continue; // do nothing if there are no features in this area

            std::sort( areas[i].surfs.begin(), areas[i].surfs.end(), sort_by_response );

            // Remove features that are too close together
            //LOGI << "Removing features that are too close together...";
            std::vector< keypoint_index_pair >& surfs = areas[i].surfs;


            size_t last = surfs.size()-1;

            for( size_t k=0; k<=last; ++k )
            {
                const cv::KeyPoint& currfeat = surfs[k].first;
                for( size_t k2=k+1; k2<=last; ++k2 )
                {
                    if( dist2d( currfeat, surfs[k2].first) < INCFG_GET(FEATURE_MIN_DISTANCE) )
                    {
                        surfs[k2] = surfs[last];
                        --last;
                        --k2;
                    }
                }
            }
            areas[i].surfs.resize( last+1 );

            if( areas[i].surfs.size() > points_per_area ) {
                areas[i].surfs.resize( points_per_area );
            }
            //LOGI << "Area " << i << ": " << areas[i].surfs.size() << " points.";
        }

        unsigned int surf_index=0;
        unsigned int area_index=0;
        unsigned int num_skip=0;
        bool more_to_add = true;

        while( more_to_add ) { //Surfs are added in this order: surf0 of area0, surf0 of area1 ... surf0 of arean, surf1 of area0, surf1 of area1 ... and so on

            if( areas[area_index].surfs.size() > surf_index ) {

                const keypoint_index_pair& fi = areas[area_index].surfs[surf_index];
                const cv::KeyPoint& f = fi.first;

                Feature newf( f.pt.x, f.pt.y, f.size, f.angle / 180.0f * M_PI );

                //fix orientation
                //if (newf.angle < 0.0f) newf.angle = 0.0f; // might happen in case of numerical errors
                //if (newf.angle > static_cast<float>(2.0 * M_PI)) newf.angle = static_cast<float>( 2.0 * M_PI );
                while( newf.angle < 0.0f )
                    newf.angle += 2.0*M_PI;
                while( newf.angle > 2*M_PI )
                    newf.angle -= 2.0*M_PI;

                newf.descriptor.resize( descriptors.cols );
                for( size_t k=0; k<64; ++k )
                {
                    newf.descriptor[k] = descriptors.at<float>( fi.second, k );
                }

                pImpl->fts.push_back( newf );

            } else {
                num_skip++;
            }

            if( num_skip == areas.size() ) {
                more_to_add = false;
            }

            area_index++;
            if( area_index == areas.size() ) {
                area_index = 0;
                num_skip=0;
                surf_index++;
            }
        }
    }

    clear_kdtree();
    LOGI << size() << " total features after resampling.";
    LOGI << "all done in " << timer.elapsed() << " secs.";
    // All done!
}


void FeatureSet::save( std::string filename ) const
{
    LOG_SCOPE("FeatureSet");
    LOGI << "Saving features...";

    if( size() < 1 )
    {
        throw FeatureExtractorException("No feature to save.");
    }

    std::ofstream ofs( filename.c_str(), std::ios::binary );
    if( ofs.fail() )
    {
        throw FeatureExtractorException("Unable to open " + filename + " for writing.");
    }

    size_t feature_size_bytes = pImpl->fts[0].size_bytes();
    boost::uint32_t n_features = ( boost::uint32_t )size();
    boost::uint32_t desc_size = ( boost::uint32_t )pImpl->fts[0].descriptor.size();

    ofs.write( (char*) &n_features, sizeof(boost::uint32_t));
    ofs.write( (char*) &desc_size, sizeof(boost::uint32_t));


    boost::scoped_array<char> buff( new char[ n_features*feature_size_bytes ] );

    for( size_t i=0; i<n_features; ++i )
    {
        pImpl->fts[i].copyBinary( (float*)(buff.get()+i*feature_size_bytes) );
    }

    ofs.write( buff.get(), n_features*feature_size_bytes );
    ofs.flush();
    ofs.close();

}


void FeatureSet::load( std::string filename )
{
    LOG_SCOPE("FeatureSet");
    LOGI << "Loading features...";
    clear();

    std::ifstream ifs( filename.c_str(), std::ios::binary );
    if( ifs.fail() )
    {
        throw FeatureExtractorException("Unable to open " + filename + " for reading.");
    }

    boost::uint32_t n_features;
    boost::uint32_t desc_size;

    ifs.read( (char*)&n_features, sizeof(boost::uint32_t) );
    ifs.read( (char*)&desc_size, sizeof(boost::uint32_t) );

    size_t feature_size_bytes = Feature::size_bytes( (size_t)desc_size );


    boost::scoped_array<char> buff( new char[ n_features*feature_size_bytes ] );

    ifs.read( buff.get(), n_features*feature_size_bytes );

    for( size_t i=0; i<n_features; ++i )
    {
        pImpl->fts.push_back( Feature((float*)(buff.get()+i*feature_size_bytes), desc_size) );
    }

    ifs.close();
}


void FeatureSet::renderToImage( cv::Mat img ) const
{
    for( unsigned int i=0; i < pImpl->areas.size(); i++ )
    {
        cv::rectangle( img, pImpl->areas[i].area, CV_RGB(0,0,0),1 );
    }

    for( size_t i=0; i<size(); ++i )
    {
        const Feature& ft = (*this)[i];
        cv::circle( img, cv::Point( static_cast<int>(ft.x()), static_cast<int>(ft.y())), static_cast<int>(std::max<float>(ft.scale*2.0f, 1.0))+1, CV_RGB(0,0,0), 3, cv::LINE_AA );
        cv::circle( img, cv::Point( static_cast<int>(ft.x()), static_cast<int>(ft.y())), static_cast<int>(std::max<float>(ft.scale*2.0f, 1.0)), CV_RGB(150,150,150), 1 );
    }

}


void FeatureSet::build_kdtree()
{
    pImpl->n_dimensions = static_cast<int>( pImpl->fts[0].descriptor.size() );
    pImpl->kdt_features = cv::Mat( static_cast<int>(pImpl->fts.size()), pImpl->n_dimensions, CV_32F );

    // Fill features matrix
    for( size_t i=0; i<size(); ++i )
    {
        for( size_t j=0; j<pImpl->n_dimensions; ++j )
        {
            pImpl->kdt_features.at<float>( static_cast<int>(i), static_cast<int>(j) ) = pImpl->fts[i].descriptor[j];
        }
    }

    pImpl->kdtree = new cv::flann::Index(pImpl->kdt_features, pImpl->indexParams);
}




void FeatureSet::clear_kdtree()
{
    pImpl->kdt_features = cv::Mat();
    pImpl->n_dimensions=-1;
    if(  pImpl->kdtree )
    {
        delete  pImpl->kdtree;
        pImpl->kdtree = 0;
    }
}



std::pair< std::vector<int>, std::vector<float> > FeatureSet::knn( const Feature& fs, const int K )
{
    if( size()==0 )
        return std::pair< std::vector<int>, std::vector<float> >( std::vector<int>(), std::vector<float>() );


    if( !pImpl->kdtree )
        build_kdtree();

    std::vector<float> singleQuery = fs.descriptor;
    std::vector<int> indices(K);
    std::vector<float> distances(K);

    pImpl->kdtree->knnSearch(singleQuery, indices, distances, K, cv::flann::SearchParams(64));

    return std::pair< std::vector<int>, std::vector<float> >( indices, distances );
}


int FeatureSet::nearest( const Feature& fs ) const
{
    int idx=-1;
    double curr_min = 1E20;

    for( size_t i=0; i<pImpl->fts.size(); ++i )
    {
        if( pImpl->fts[i] == fs )   // Avoid returning itself
            continue;

        double distance = fs.spatial_distance( pImpl->fts[i] );
        if( distance<curr_min )
        {
            curr_min = distance;
            idx = static_cast<int>( i );
        }
    }
    return idx;
}

int FeatureSet::nearest( const int fs_idx ) const
{
    return nearest( pImpl->fts.at(fs_idx) );
}
