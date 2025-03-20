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

#define _USE_MATH_DEFINES
#include <cmath>
#include "GTMatcher.h"
#include "incfg.hpp"
#include "hires_timer.h"
#include "log.hpp"
#include "gt.h"
#include <boost/cstdint.hpp>
#include <boost/scoped_array.hpp>
#include <fstream>


INCFG_REQUIRE( float, NNDR, 0.25f, "Nearest neighbour distance ratio used to select best matches. Note: ignored if MATCHER_SKIP_GT=false" )


#undef max
#undef min
#define USE_ANGLE_DIFFERENCE


using WASS::match::Match;
using WASS::match::GTMatcher;
using WASS::match::MatchList;
using WASS::match::FeatureSet;
using WASS::match::Feature;

inline double ang_diff( double firstAngle, double secondAngle )
{
    double difference = secondAngle - firstAngle;
    while (difference < -M_PI) difference += 2.0*M_PI;
    while (difference > M_PI) difference -= 2.0*M_PI;
    return difference;
}


struct association
{
   int source;
   int target;

   std::vector<float> source_spectrum;
   std::vector<float> dest_spectrum;

   double sin_alpha, cos_alpha, delta_x, delta_y, delta_scale;

   explicit association(int s, int t) : source(s), target(t) {}
   explicit association() : source(-1), target(-1) {}

   void compute_affine( const FeatureSet& fs_source, const FeatureSet& fs_target )
   {
       const Feature& src = fs_source[ source ];
       const Feature& targ = fs_target[ target ];

       const double a = targ.angle;
       const double b = src.angle;

#ifdef USE_ANGLE_DIFFERENCE
       const double deltaRot = ang_diff(a,b);
#else
       const double deltaRot = a - b;
#endif

       const double cosAlfa = std::cos(deltaRot);
       const double sinAlfa = std::sin(deltaRot);
       const double deltaScale = (double)targ.scale/(double)src.scale;

       const double scaledX = src.position[0]*deltaScale;
       const double scaledY = src.position[1]*deltaScale;
       const double x = scaledX * cosAlfa - scaledY * sinAlfa;
       const double y = scaledX * sinAlfa + scaledY * cosAlfa;

       cos_alpha = cosAlfa;
       sin_alpha = sinAlfa;
       delta_x = targ.position[0] - x;
       delta_y = targ.position[1] - y;
       delta_scale = deltaScale;
   }
};


inline double geometric_error( const association& a1, const association& a2,
                               const FeatureSet& f1,
                               const FeatureSet& f2 )
{
    const Feature& s1 = f1[a1.source];
    const Feature& s2 = f1[a2.source];
    const Feature& t1 = f2[a1.target];
    const Feature& t2 = f2[a2.target];

    //const float min_feat_dist = 5.0f;
    //if (fabs(s1.x - s2.x) <= min_feat_dist && fabs(s1.y - s2.y) <= min_feat_dist &&
    //    fabs(t1.x - t2.x) <= min_feat_dist && fabs(t1.y - t2.y) <= min_feat_dist)
    //{ return 0.; }

    // apply Tx1 to S2
    const double errorX = t2.x() - (a1.delta_scale * (s2.x() * a1.cos_alpha - s2.y() * a1.sin_alpha) + a1.delta_x);
    const double errorY = t2.y() - (a1.delta_scale * (s2.x() * a1.sin_alpha + s2.y() * a1.cos_alpha) + a1.delta_y);

    // apply Tx2 to S1
    const double errorX2 = t1.x() - (a2.delta_scale * (a2.cos_alpha * s1.x() - a2.sin_alpha * s1.y()) + a2.delta_x);
    const double errorY2 = t1.y() - (a2.delta_scale * (a2.sin_alpha * s1.x() + a2.cos_alpha * s1.y()) + a2.delta_y);

    return std::max(errorX * errorX + errorY * errorY, errorX2 * errorX2 + errorY2 * errorY2);
}



inline double payoff(
      const association& a1, const association& a2,
      const FeatureSet& f1,
      const FeatureSet& f2,
      double payoff_lambda_affine )
{

   if(a1.source == a2.source || a1.target == a2.target)
      return 0.0;

   double ge = geometric_error(a1,a2,f1,f2);

   return std::exp( -payoff_lambda_affine * ge);
}




namespace WASS
{
namespace match
{
    class GTMatcherImpl
    {
        public:

        explicit GTMatcherImpl( FeatureSet& _a, FeatureSet& _b ) : fa(_a), fb(_b) {}
        FeatureSet& fa;
        FeatureSet& fb;
        std::vector<double> payoff_matrix;
        std::vector< association > cmatches;
    };
}
}



GTMatcher::GTMatcher( FeatureSet& _fa, FeatureSet& _fb  )
{

    n_candidates_per_feature = 3; // Just a default initial value
    pImpl = new GTMatcherImpl( _fa, _fb );
}


GTMatcher::~GTMatcher()
{
    delete pImpl;
    pImpl=0;
}


MatchList GTMatcher::generate_candidates( )
{
    MatchList ml;
    pImpl->cmatches.clear();
    pImpl->payoff_matrix.clear();

    for( size_t i=0; i<pImpl->fa.size(); ++i )
    {
        const WASS::match::Feature& fs = pImpl->fa[i];
        auto neighbours = pImpl->fb.knn( fs,n_candidates_per_feature );

        const std::vector< int >& nn = neighbours.first;
        const std::vector< float >& distances = neighbours.second;

        // Distance ratio euristic
        // skip this match if the second neighbour distance
        // is too close to the best neighbour
        if( n_candidates_per_feature>1 && (distances[0] < INCFG_GET(NNDR) * distances[1]) )
        {
            Match p;
            p.first = (size_t)i;
            p.second = (size_t)nn[0];
            p.imga_loc = pImpl->fa[p.first].position;
            p.imgb_loc = pImpl->fb[p.second].position;
            ml.push_back( p );
        }


        for( size_t j=0; j<nn.size(); ++j )
        {
            pImpl->cmatches.push_back( association( (int)i,
                                                     nn[j]) );
        }
    }

    return ml;
}


void GTMatcher::compute_payoff_matrix( double payoff_lambda_geometric )
{
    LOG_SCOPE("GTMatcher");

    cvlab::HiresTimer timer;
    timer.start();

    LOGI <<  pImpl->cmatches.size() << " total candidates.";
    LOGI << " computing payoff matrix...";

    int Ncandidates = (int)pImpl->cmatches.size();

    for( int i=0; i<Ncandidates; ++i )
    {
        pImpl->cmatches[i].compute_affine( pImpl->fa, pImpl->fb );
    }


    pImpl->payoff_matrix.resize( Ncandidates*Ncandidates );
    for( int i=0; i<Ncandidates; ++i )
    {
        for( int j=0; j<Ncandidates; ++j )
        {
            const double x = payoff(pImpl->cmatches[i], pImpl->cmatches[j], pImpl->fa, pImpl->fb, payoff_lambda_geometric );
            pImpl->payoff_matrix[i*Ncandidates + j] = x;
            pImpl->payoff_matrix[j*Ncandidates + i] = pImpl->payoff_matrix[i*Ncandidates + j];

        }
    }

    LOGI << "payoff matrix generated in " << timer.elapsed() << " secs.";
}


MatchList GTMatcher::match_group( double pop_threshold )
{
    LOG_SCOPE("GTMatcher");

    cvlab::HiresTimer timer;
    timer.start();

    MatchList matches;
    int N = (int)pImpl->cmatches.size();
    std::vector< double > population(N);
    gt_create_population( &(population[0]), N);

    LOGI << "running non-cooperative game...";

    double toll = 1E-20;
    int iters = 50000;
    gt_iidyn( &(pImpl->payoff_matrix[0]),&(population[0]), N, &toll, &iters );

    LOGI << "game completed after " <<  iters << " iterations, toll=" << toll;

    const double population_thresh = *std::max_element(population.begin(), population.end()) * pop_threshold;

    LOGI << "population threshold value " << population_thresh;

    std::vector< size_t > winning_src_features;
    std::vector< size_t > winning_dst_features;

    for( size_t i=0; i<N; ++i )
    {
        if( population[i]>population_thresh )
        {
            Match p;
            p.first = (size_t) pImpl->cmatches[i].source;
            p.second = (size_t) pImpl->cmatches[i].target;
            p.imga_loc = pImpl->fa[p.first].position;
            p.imgb_loc = pImpl->fb[p.second].position;

            matches.push_back( p );
            winning_src_features.push_back( p.first );
            winning_dst_features.push_back( p.second );
        }
    }

    LOGI << "number of winning strategies: " << winning_src_features.size();


    // remove all candidates containing winning features
    for( size_t i=0; i<pImpl->cmatches.size(); ++i )
    {
        for( size_t j=0; j<winning_src_features.size(); ++j )
        {
            size_t src_idx = winning_src_features[j];
            size_t dst_idx = winning_dst_features[j];

            if( pImpl->cmatches[i].source == src_idx || pImpl->cmatches[i].target == dst_idx )
            {
                // Remove this candidate
                pImpl->cmatches[i] = pImpl->cmatches.back();
                pImpl->cmatches.resize( pImpl->cmatches.size()-1 );
                break;
            }
        }
    }

    LOGI << pImpl->cmatches.size() << " remaining candidates";
    LOGI << "GT inlier selection performed in " << timer.elapsed() << " secs.";

    return matches;
}


void GTMatcher::save_matches( std::string filename, const MatchList& matches )
{
    std::ofstream ofs( filename.c_str(), std::ios::binary );
    if( ofs.fail() )
    {
        throw GTMatcherException(std::string("Unable to save matches to ")+filename);
    }

    boost::uint32_t n_matches = (boost::uint32_t)matches.size();
    ofs.write( (char*)&n_matches, sizeof(boost::uint32_t) );


    boost::scoped_array< char > file_data( new char[matches.size()*sizeof(Match)] );

    char* pDataPtr = file_data.get();

    for( size_t i=0; i<matches.size(); ++i )
    {
        memcpy( pDataPtr, (char*)&(matches[i]), sizeof(Match) );
        pDataPtr += sizeof(Match);
    }

    ofs.write( file_data.get(), matches.size()*sizeof(Match) );
    ofs.flush();
    ofs.close();
}


void GTMatcher::load_matches( std::string filename, MatchList& matches )
{
    std::ifstream ifs( filename.c_str(), std::ios::binary );
    if( ifs.fail() )
    {
        throw GTMatcherException(std::string("Unable to load matches from ")+filename);
    }

    boost::uint32_t n_matches;
    ifs.read( (char*)&n_matches, sizeof(boost::uint32_t) );

    matches.resize( n_matches );
    for( size_t i=0; i<matches.size(); ++i )
    {
        ifs.read( (char*)&(matches[i]), sizeof(Match) );
    }

    ifs.close();
}
