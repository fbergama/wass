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


#ifndef _FEATURE_SET_H_
#define _FEATURE_SET_H_

#include "ImageFeature.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/config.hpp>


namespace WASS
{
namespace match
{

class FeatureSetImpl;

struct SURF_Extractor_params
{
    int n_octaves;
    int n_octave_layers;
    int init_samples;
    float hessian_thresh;
    static  SURF_Extractor_params get_default();
};


inline std::ostream& operator<<( std::ostream& os, const SURF_Extractor_params& p )
{
    os << "SURF Extractor params:" << std::endl;
    os << "    n octaves: " << p.n_octaves << std::endl;
    os << "     n layers: " << p.n_octave_layers << std::endl;
    os << "    init samp: " << p.init_samples << std::endl;
    os << "  hess thresh: " << p.hessian_thresh << std::endl;
    return os;
}


class FeatureSet
{

public:

    class FeatureExtractorException : public std::exception
    {
    public:
        inline FeatureExtractorException( std::string _wh ) : wh(_wh) {}
        virtual const char* what() const BOOST_NOEXCEPT { return wh.c_str(); }

    private:
        std::string wh;
    };

    FeatureSet();
    virtual ~FeatureSet();
    virtual Feature& operator[]( size_t idx ) const;
    virtual size_t size() const;

    virtual void clear();

    virtual void detect( cv::Mat img, size_t max_features, SURF_Extractor_params prms );

    virtual void save( std::string filename ) const;
    virtual void load( std::string filename );

    virtual void renderToImage( cv::Mat img ) const;


    std::pair< std::vector< int >, std::vector<float> > knn( const Feature& fs, const int K ); // Returns the k nearest features with respect to descriptor

    int nearest( const Feature& fs ) const; // Returns the nearest feature with respect to image position
    int nearest( const int fs_idx ) const;  // the same as above, but uses the feature index as input parameter

private:
    FeatureSetImpl* pImpl;

    void build_kdtree();
    void clear_kdtree();
};

}}

#endif
