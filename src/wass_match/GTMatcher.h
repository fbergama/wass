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


#ifndef _GT_MATCHER_H_
#define _GT_MATCHER_H_


#include "FeatureSet.h"
#include <vector>
#include <boost/config.hpp>


namespace WASS
{
namespace match
{

struct Match
{
public:
    size_t first;
    size_t second;
    cv::Vec2f imga_loc;
    cv::Vec2f imgb_loc;
};

typedef std::vector< Match > MatchList;





class GTMatcherImpl;
class GTMatcher
{
public:
    class GTMatcherException : public std::exception
    {
    public:
        inline GTMatcherException( std::string _wh ) : wh(_wh) {}
        virtual const char* what() const BOOST_NOEXCEPT { return wh.c_str(); }

    private:
        std::string wh;
    };

    explicit GTMatcher( FeatureSet& _fa, FeatureSet& _fb  );
    ~GTMatcher();

    MatchList generate_candidates( );
    void compute_payoff_matrix( double payoff_lambda_geometric );
    MatchList match_group( double pop_threshold );

    int n_candidates_per_feature;

    static void save_matches( std::string filename, const MatchList& matches );
    static void load_matches( std::string filename, MatchList& matches );

private:
    GTMatcherImpl* pImpl;
};

}
}

#endif
