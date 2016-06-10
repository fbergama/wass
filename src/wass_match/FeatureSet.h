#ifndef _FEATURE_SET_H_
#define _FEATURE_SET_H_

#include "ImageFeature.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


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
        virtual const char* what() const noexcept { return wh.c_str(); }

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


    std::vector< int > knn( const Feature& fs, const int K ); // Returns the k nearest features with respect to descriptor

    int nearest( const Feature& fs ) const; // Returns the nearest feature with respect to image position
    int nearest( const int fs_idx ) const;  // the same as above, but uses the feature index as input parameter

private:
    FeatureSetImpl* pImpl;

    void build_kdtree();
    void clear_kdtree();
};

}}

#endif
